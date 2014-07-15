// Do not remove the include below
#include "lintilla_embedded.h"

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

#include "LintillaMmi.h"
#include "ALintillaMmiAdapter.h"
#include "Timer.h"
#include "SN754410Driver.h"
#include "MotorPWM.h"
#include "UltrasonicSensor.h"
#include "UltrasonicSensorHCSR04.h"
#include "EEPROM.h"
#include "Ivm.h"
#include "IF_IvmMemory.h"
#include "IvmSerialEeprom.h"
#include "LintillaIvm.h"
#include "CmdAdapter.h"
#include "CmdSequence.h"
#include "Cmd.h"
#include "DistanceCount.h"
#include "Battery.h"
#include "LintillaBatteryAdapter.h"
//#include <aJSON.h>

//-----------------------------------------------------------------------------

void(* resetFunc) (void) = 0; //declare reset function at address 0 => will 'use' NULL pointer exception and then restart.

//-----------------------------------------------------------------------------
// Debugging
//-----------------------------------------------------------------------------
Timer* ramDebugTimer = 0;
const unsigned int c_ramDbgInterval = 20000;
class RamDebugTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  }
};

//---------------------------------------------------------------------------
// Inventory Management
//---------------------------------------------------------------------------
LintillaIvm* ivm = 0;

//-----------------------------------------------------------------------------
// Battery Voltage Surveillance
//-----------------------------------------------------------------------------
Battery* battery = 0;
LintillaBatteryAdapter* batteryAdapter = 0;

//---------------------------------------------------------------------------
// Ultrasonic Ranging
//---------------------------------------------------------------------------
UltrasonicSensor* ultrasonicSensorFront = 0;

const unsigned int triggerPin = 34;
const unsigned int echoPin    = 36;
unsigned long dist = UltrasonicSensor::DISTANCE_LIMIT_EXCEEDED;   // [cm]

//---------------------------------------------------------------------------
// Wheel Speed Sensors
//---------------------------------------------------------------------------
const unsigned int SPEED_SENSORS_READ_TIMER_INTVL_MILLIS = 100;
Timer* speedSensorReadTimer = 0;
void readSpeedSensors();
class SpeedSensorReadTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    readSpeedSensors();
  }
};

const int IRQ_PIN_18 = 5;
const int IRQ_PIN_19 = 4;
const int IRQ_PIN_20 = 3;
const int IRQ_PIN_21 = 2;

const int L_SPEED_SENS_IRQ = IRQ_PIN_18;
const int R_SPEED_SENS_IRQ = IRQ_PIN_19;

volatile unsigned long int speedSensorCountLeft  = 0;
volatile unsigned long int speedSensorCountRight = 0;

volatile long int leftWheelSpeed  = 0;
volatile long int rightWheelSpeed = 0;

void countLeftSpeedSensor();
void countRightSpeedSensor();

//---------------------------------------------------------------------------
// Distance Counters
//---------------------------------------------------------------------------
DistanceCount* lDistCount = 0;
DistanceCount* rDistCount = 0;

//---------------------------------------------------------------------------
// Motor Drivers and Speed Control
//---------------------------------------------------------------------------
MotorPWM* motorL = 0;
MotorPWM* motorR = 0;

const int cSpeed     = 200;
const int cSpinSpeed = 150;

// H-bridge enable pin for speed control
const int speedPin1 = 44;
const int speedPin2 = 45;

// H-bridge leg 1
const int motor1APin = 46;
const int motor3APin = 47;

// H-bridge leg 2
int motor2APin = 48;
int motor4APin = 49;

// value for motor speed
int speed_value_motor_left  = 0;
int speed_value_motor_right = 0;
bool isLeftMotorFwd = true;
bool isRightMotorFwd = true;
bool isObstacleDetected = false;

void motorStop();
void moveBackward();
void moveForward();
void moveStraight(bool forward);
void spinOnPlace(bool right);

Timer* speedCtrlTimer = 0;
const unsigned int cSpeedCtrlInterval = 200;
void speedControl();
void updateActors();
class SpeedCtrlTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    speedControl();
    updateActors();
  }
};

//---------------------------------------------------------------------------
// Command Sequence
//---------------------------------------------------------------------------
CmdSequence* cmdSeq = 0;
class LintillaCmdAdapter;

//---------------------------------------------------------------------------
// WiFi and Socket Server
//---------------------------------------------------------------------------
void startEchoServer();
void processEchoServer();
//aJsonStream* jsonStream = 0;

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  12 // WEN, was on  5 before
#define ADAFRUIT_CC3000_CS    13 // WCS, was on 10 before
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2); // you can change this clock speed but DI

#define WLAN_SSID       "LintillaNet"        // cannot be longer than 32 characters!
#define WLAN_PASS       "AnswerIs42"
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define LISTEN_PORT     9999    // What TCP port to listen on for connections.  The echo protocol uses port 7.

uint32_t currentIpAddress = 0;
Adafruit_CC3000_Server echoServer(LISTEN_PORT);

Timer* wifiReconnectTimer = 0;
const unsigned int cWifiReconnectInterval = 5000; // WiFi re-connect interval [ms]
void connectWiFi();
class WifiReconnectTimerAdapter : public TimerAdapter
{
  void timeExpired()
  {
    if (!cc3000.checkConnected())
    {
      Serial.print("Lintilla lost WiFi connection, rebooting!!\n");
//        Serial.print("Lintilla lost WiFi connection, reconnecting!!\n");
      delay(2000);
      resetFunc();
//      connectWiFi();
//      startEchoServer();
    }
  }
};

bool displayConnectionDetails(void);
bool isSSIDPresent(const char* searchSSID);

//---------------------------------------------------------------------------
// Lintilla MMI
//---------------------------------------------------------------------------
LintillaMmi* mmi = 0;

//-----------------------------------------------------------------------------

class LintillaCmdAdapter : public CmdAdapter
{
  virtual void stopAction()
  {
    motorStop();
    Serial.print("LintillaCmdAdapter::stopAction()\n");
  }

  virtual void moveForwardAction()
  {
    moveForward();
    Serial.print("LintillaCmdAdapter::moveForwardAction()\n");
  }

  virtual void moveBackwardAction()
  {
    moveBackward();
    Serial.print("LintillaCmdAdapter::moveBackwardAction()\n");
  }

  virtual void spinOnPlaceLeftAction()
  {
    spinOnPlace(false);
    Serial.print("LintillaCmdAdapter::spinOnPlaceLeftAction()\n");
  }

  virtual void spinOnPlaceRightAction()
  {
    spinOnPlace(true);
    Serial.print("LintillaCmdAdapter::spinOnPlaceRightAction()\n");
  }
};

//-----------------------------------------------------------------------------

void connectWiFi()
{
  Serial.println(F("\nHello, CC3000!"));
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  /* Initialize the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }

  /* Delete any old connection data on the module */
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    return; // bail out
  }

  Serial.print(F("Scanning for SSID ")); Serial.println(WLAN_SSID);
  if (!isSSIDPresent(WLAN_SSID))
  {
    Serial.println(F("Failed!"));
    return;  // bail out
  }
  Serial.println(F("Succeeded."));

  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY))
  {
    // time out after 10 s
    Serial.println(F("Failed!"));
    return;  // bail out
  }
  Serial.println(F("Connected!"));

  Serial.println(F("Request DHCP"));
  const unsigned int dhcpConnectRetryInterval = 1000; // [s]
  unsigned int dhcpTimoutCounter = 20;                // 10 s
  while ((dhcpTimoutCounter > 0) && !cc3000.checkDHCP())
  {
    dhcpTimoutCounter--;
    delay(dhcpConnectRetryInterval);
  }
  if (0 == dhcpTimoutCounter)
  {
    return; // bail out
  }

  /* Display the IP address DNS, Gateway, etc. */
  while (!displayConnectionDetails())
  {
    delay(1000);
  }
}

//-----------------------------------------------------------------------------

void countLeftSpeedSensor()
{
  noInterrupts();
  speedSensorCountLeft++;
  interrupts();
}

//-----------------------------------------------------------------------------

void countRightSpeedSensor()
{
  noInterrupts();
  speedSensorCountRight++;
  interrupts();
}

//-----------------------------------------------------------------------------

/**************************************************************************/
/*!
    @brief  Tries to read the IP address and other connection details
*/
/**************************************************************************/
bool displayConnectionDetails(void)
{
  uint32_t ipAddress, netmask, gateway, dhcpserv, dnsserv;

  if(!cc3000.getIPAddress(&ipAddress, &netmask, &gateway, &dhcpserv, &dnsserv))
  {
    Serial.println(F("Unable to retrieve the IP Address!\r\n"));
    return false;
  }
  else
  {
    Serial.print(F("\nIP Addr: ")); cc3000.printIPdotsRev(ipAddress);
    Serial.print(F("\nNetmask: ")); cc3000.printIPdotsRev(netmask);
    Serial.print(F("\nGateway: ")); cc3000.printIPdotsRev(gateway);
    Serial.print(F("\nDHCPsrv: ")); cc3000.printIPdotsRev(dhcpserv);
    Serial.print(F("\nDNSserv: ")); cc3000.printIPdotsRev(dnsserv);
    Serial.println();
    currentIpAddress = ipAddress;
    return true;
  }
}

//-----------------------------------------------------------------------------

bool isSSIDPresent(const char* searchSSID)
{
  bool found = false;
  uint8_t valid, rssi, sec, index;
  char ssidname[33];

  index = cc3000.startSSIDscan();

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while ((index > 0) && !found)
  {
    index--;
    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);

    if (0 == strncmp(ssidname, searchSSID, sizeof(ssidname)))
    {
      found = true;
    }

    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    if (found)
    {
      Serial.print(" <found>");
    }
    Serial.println();
    Serial.print(F("RSSI         : "));
    Serial.println(rssi);
    Serial.print(F("Security Mode: "));
    Serial.println(sec);
    Serial.println();
  }

  Serial.println(F("================================================"));

  cc3000.stopSSIDscan();
  return found;
}

//-----------------------------------------------------------------------------

// The loop function is called in an endless loop
void loop()
{
  yield();
//  processEchoServer();
}

//-----------------------------------------------------------------------------

void motorStop()
{
  speed_value_motor_left  = 0;
  speed_value_motor_right = 0;
  updateActors();
}

//-----------------------------------------------------------------------------

void moveBackward()
{
  moveStraight(false);
}

//-----------------------------------------------------------------------------

void moveForward()
{
  moveStraight(true);
}

//-----------------------------------------------------------------------------

void moveStraight(bool forward)
{
  isLeftMotorFwd  = forward;
  isRightMotorFwd = forward;
  speed_value_motor_left  = cSpeed;
  speed_value_motor_right = cSpeed;
  updateActors();
}

//-----------------------------------------------------------------------------

void processEchoServer()
{
  if (cc3000.checkConnected())
  {
    //  if (jsonStream->available()) {
    //    /* First, skip any accidental whitespace like newlines. */
    //    jsonStream->skip();
    //  }

    //  if (jsonStream->available()) {
    //    /* Something real on input, let's take a look. */
    //    aJsonObject* msg = aJson.parse(jsonStream);
    //    processLintillaMessageReceived(msg);
    //    aJson.deleteItem(msg);
    //  }

    // Try to get a client which is connected.
    Adafruit_CC3000_ClientRef client = echoServer.available();

    if (client) {
       // Check if there is data available to read.
       if (client.available() > 0)
       {
         // Read a byte and write it to all clients.
         uint8_t ch = client.read();
         if (('g' == ch) && !cmdSeq->isRunning() /*&& !lcdKeypad->isRightKey()*/ && !isObstacleDetected &&
             !battery->isBattVoltageBelowStopThreshold() && !battery->isBattVoltageBelowShutdownThreshold())
         {
           cmdSeq->start();
         }
         else if ('h' == ch)
         {
           cmdSeq->stop();
         }
         client.write(ch);
       }
    }
  }
}

//-----------------------------------------------------------------------------


void readSpeedSensors()
{
  noInterrupts();

  // read the speed sensor counters and reset them
  leftWheelSpeed  = speedSensorCountLeft;  speedSensorCountLeft  = 0;
  rightWheelSpeed = speedSensorCountRight; speedSensorCountRight = 0;

  lDistCount->add(leftWheelSpeed);
  rDistCount->add(rightWheelSpeed);

  interrupts();
}

//-----------------------------------------------------------------------------

//The setup function is called once at startup of the sketch
void setup()
{
  //---------------------------------------------------------------------------
  // Debugging
  //---------------------------------------------------------------------------
  Serial.begin(115200);
  Serial.println(F("\nHello from Lintilla!"));
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  ramDebugTimer = new Timer(new RamDebugTimerAdapter(), Timer::IS_RECURRING, c_ramDbgInterval);

  //---------------------------------------------------------------------------
  // Inventory Management
  //---------------------------------------------------------------------------
//  #include "EEPROM.h"
//  EEPROM.write(1, 1);
  ivm = new LintillaIvm();

  //---------------------------------------------------------------------------
  // Battery Voltage Surveillance
  //---------------------------------------------------------------------------
//  Serial.println("IVM:");
//  Serial.println("---------------------------------------------------");
//  Serial.print("ID: "); Serial.println(ivm->getDeviceId());
//  Serial.print("IVM V."); Serial.println(ivm->getIvmVersion());
//  Serial.print("BattVoltSenseFactor = "); Serial.println(ivm->getBattVoltageSenseFactor());
  batteryAdapter = new LintillaBatteryAdapter();
  battery = new Battery(batteryAdapter);
  batteryAdapter->attachBattery(battery);
  batteryAdapter->attachLintillaMmi(mmi);
  batteryAdapter->attachLintillaIvm(ivm);
  batteryAdapter->attachCmdSequence(cmdSeq);

  //---------------------------------------------------------------------------
  // Ultrasonic Ranging
  //---------------------------------------------------------------------------
  ultrasonicSensorFront = new UltrasonicSensorHCSR04(triggerPin, echoPin);

  //---------------------------------------------------------------------------
  // Speed Sensors
  //---------------------------------------------------------------------------
//  speedSensorReadTimer  = new Timer(new SpeedSensorReadTimerAdapter(), Timer::IS_RECURRING, SPEED_SENSORS_READ_TIMER_INTVL_MILLIS);
//  attachInterrupt(L_SPEED_SENS_IRQ, countLeftSpeedSensor,  RISING);
//  attachInterrupt(R_SPEED_SENS_IRQ, countRightSpeedSensor, RISING);

  //---------------------------------------------------------------------------
  // Distance Counters
  //---------------------------------------------------------------------------
  lDistCount = new DistanceCount();
  rDistCount = new DistanceCount();

  //---------------------------------------------------------------------------
  // Motor Drivers and Speed Control
  //---------------------------------------------------------------------------
  motorL = new SN754410_Driver(speedPin1, motor1APin, motor2APin);
  motorR = new SN754410_Driver(speedPin2, motor3APin, motor4APin);
  speedCtrlTimer = new Timer(new SpeedCtrlTimerAdapter(), Timer::IS_RECURRING, cSpeedCtrlInterval);

  //---------------------------------------------------------------------------
  // Command Sequence
  //---------------------------------------------------------------------------
  cmdSeq = new CmdSequence(new LintillaCmdAdapter());

  const int cSpinTime     =  300;
  const int cFwdTime      =  300;
  const int cInterDelay   =  500;

  for (int i = 0; i <= 3; i++)
  {
    new CmdMoveForward(cmdSeq, cFwdTime);
    new CmdStop(cmdSeq, cInterDelay);
    new CmdSpinOnPlaceRight(cmdSeq, cSpinTime);
    new CmdStop(cmdSeq, cInterDelay);
  }

  Cmd* cmd = cmdSeq->getFirstCmd();
  while (0 != cmd)
  {
    Serial.print("cmdSeq: ");
    Serial.print(cmd->getName());
    Serial.print("; t=");
    Serial.print(cmd->getTime());
    Serial.println("[ms]");
    cmd = cmdSeq->getNextCmd();
  }

  //---------------------------------------------------------------------------
  // WiFi and Socket Server
  //---------------------------------------------------------------------------
//  connectWiFi();
//  startEchoServer();
//  wifiReconnectTimer = new Timer(new WifiReconnectTimerAdapter(), Timer::IS_RECURRING, cWifiReconnectInterval);

  //---------------------------------------------------------------------------
  // MMI
  //---------------------------------------------------------------------------
  mmi = new LintillaMmi(new ALintillaMmiAdapter(cmdSeq, ivm, ultrasonicSensorFront, &cc3000, lDistCount, rDistCount));
}

//-----------------------------------------------------------------------------

void speedControl()
{
  if (0 != ultrasonicSensorFront)
  {
    dist = ultrasonicSensorFront->getDistanceCM();
  }
  isObstacleDetected = isLeftMotorFwd && (dist > 0) && (dist < 15);

//  if (lcdKeypad->isRightKey() || isObstacleDetected || battery->isBattVoltageBelowStopThreshold() || battery->isBattVoltageBelowShutdownThreshold())
//  {
//    cmdSeq->stop();
//  }
}

//-----------------------------------------------------------------------------

void spinOnPlace(bool right)
{
  isLeftMotorFwd = right;
  isRightMotorFwd = !right;
  speed_value_motor_left  = cSpinSpeed;
  speed_value_motor_right = cSpinSpeed;
  updateActors();
}

//-----------------------------------------------------------------------------

void startEchoServer()
{
  if (cc3000.checkConnected())
  {
    // jsonStream = new aJsonStream(&(Stream)echoServer.available());

    // Start listening for connections
    echoServer.begin();
    Serial.println(F("Listening for connections..."));
  }
}

//-----------------------------------------------------------------------------

void updateActors()
{
  int speedAndDirectionLeft  = speed_value_motor_left  * (isLeftMotorFwd  ? 1 : -1);
  int speedAndDirectionRight = speed_value_motor_right * (isRightMotorFwd ? 1 : -1);

  motorL->setSpeed(speedAndDirectionLeft);
  motorR->setSpeed(speedAndDirectionRight);
}

//-----------------------------------------------------------------------------

/* Process message like: {"straight":{"distance": 10,"topspeed": 100},"turn":{"angle": 45},"emergencyStop":{"stop":false}}*/
//void processLintillaMessageReceived(aJsonObject *msg)
//{
////  bool emergencyStopValue = false;
////  int topspeed = 0;
////  int distanceValue = 0;
////  int angleValue = 0;
//  double batteryVoltage = 0.0;
//
//  /* Lintilla Command List Example
//  {
//      "commands": [
//          {
//              "straight": {
//                  "distance": 10,
//                  "topspeed": 100
//              }
//          },
//          {
//              "turn": {
//                  "angle": 45
//              }
//          },
//          {
//              "straight": {
//                  "distance": 20,
//                  "topspeed": 50
//              }
//          },
//          {
//              "emergencyStop": null
//          },
//          {
//              "readVoltage": null
//          }
//      ]
//  }
//  */
//
//  aJsonObject* commands = aJson.getObjectItem(msg, "commands");
//  if (0 == commands)
//  {
//    Serial.println("NO commands");
//  }
//  else
//  {
//    Serial.println("commands");
//
//    aJsonObject* aCommand = commands->child;
//    while (0 != aCommand)
//    {
//      aJsonObject* readVoltageCmd = aJson.getObjectItem(aCommand, "readVoltage");
//      if (0 != readVoltageCmd)
//      {
//        batteryVoltage = battVoltage;
//
//        aJsonObject* readVoltageMsgRoot = aJson.createObject();
//        aJsonObject* readVoltageMsgElem = aJson.createObject();
//
//        aJson.addItemToObject(readVoltageMsgRoot, "readVoltage", readVoltageMsgElem);
//        aJson.addNumberToObject(readVoltageMsgElem, "voltage", batteryVoltage);
//
//        aJson.print(readVoltageMsgRoot, jsonStream);
//        Serial.println(); /* Add newline. */
//
//        aJson.deleteItem(readVoltageMsgElem);
//        aJson.deleteItem(readVoltageMsgRoot);
//      }
//      aJson.deleteItem(readVoltageCmd);
//
//      aJsonObject* straightCmd = aJson.getObjectItem(aCommand, "straight");
//      if (0 != straightCmd)
//      {
//        Serial.println("straight");
//      }
//      aJson.deleteItem(straightCmd);
//
//      aCommand = aCommand->next;
//    }
//  }
//}

/*
 SerialEvent occurs whenever a new data comes in the
 hardware serial RX.  This routine is run between each
 time loop() runs, so using delay inside loop can delay
 response.  Multiple bytes of data may be available.
 */
void serialEvent()
{
  while (Serial.available())
  {
    // get the new byte:
    char inChar = (char)Serial.read();
    if ('i' == inChar)
    {
      batteryAdapter->incrRawBattSenseValue();
    }
    else if ('d' == inChar)
    {
      batteryAdapter->decrRawBattSenseValue();
    }
  }
}
