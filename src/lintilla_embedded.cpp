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
#include "Traction.h"
#include "ATractionAdapter.h"
#include "SpeedSensors.h"
#include "UltrasonicSensor.h"
#include "UltrasonicSensorHCSR04.h"
#include "AnUltrasonicSensorAdapter.h"
#include "EEPROM.h"
#include "Ivm.h"
#include "LintillaIvm.h"
#include "ACmdAdapter.h"
#include "CmdSequence.h"
#include "Cmd.h"
#include "DistanceCount.h"
#include "Battery.h"
#include "LintillaBatteryAdapter.h"

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
SpeedSensors* speedSensors = 0;

//---------------------------------------------------------------------------
// Motor Drivers and Speed Control
//---------------------------------------------------------------------------
Traction* traction = 0;
ATractionAdapter* tractionAdapter = 0;

//---------------------------------------------------------------------------
// Command Sequence
//---------------------------------------------------------------------------
CmdSequence* cmdSeq = 0;

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
const unsigned int cWifiReconnectInterval = 30000; // WiFi re-connect interval [ms]
void connectWiFi();
class WifiReconnectTimerAdapter : public TimerAdapter
{
private:
  Timer* m_timer;

public:
  WifiReconnectTimerAdapter(Timer* timer)
  : m_timer(timer)
  { }

  void timeExpired()
  {
    if (!cc3000.checkConnected())
    {
      Serial.print("Lintilla lost WiFi connection, reconnecting!!\n");
      delay(2000);
      cc3000.reboot();
      connectWiFi();
      startEchoServer();
    }
  }

private: // forbidden default functions
  WifiReconnectTimerAdapter& operator= (const WifiReconnectTimerAdapter& src);  // assignment operator
  WifiReconnectTimerAdapter(const WifiReconnectTimerAdapter& src);              // copy constructor
};

void connectWiFi();
uint16_t checkFirmwareVersion(void);
void displayMACAddress(void);
bool displayConnectionDetails(void);
bool isSSIDPresent(const char* searchSSID);

//---------------------------------------------------------------------------
// Lintilla MMI
//---------------------------------------------------------------------------
LintillaMmi* mmi = 0;

//-----------------------------------------------------------------------------

void connectWiFi()
{
  Serial.println(F("\nconnectWifi(): using CC3000 driver!"));
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);

  /* Initialize the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    return; // bail out
  }

  uint16_t firmware = checkFirmwareVersion();
  if (firmware < 0x113) {
    Serial.println(F("Wrong CC3000 firmware version!"));
    return; // bail out
  }

  displayMACAddress();

  /* Attempt to connect to an access point */
  char* ssid = WLAN_SSID;             /* Max 32 chars */
  Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);

  if (!isSSIDPresent(WLAN_SSID))
  {
    Serial.println(F("Failed!"));
    return;  // bail out
  }

  /* Delete any old connection data on the module */
  Serial.println(F("\nDeleting old connection profiles"));
  if (!cc3000.deleteProfiles()) {
    Serial.println(F("Failed!"));
    return; // bail out
  }

  /* Attempt to connect to an access point */
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY))
  {
    // time out after 10 s
    Serial.println(F("Failed!"));
    return;  // bail out
  }
  Serial.println(F("Connected!"));

  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  const unsigned int dhcpConnectRetryInterval = 1000; // [ms]
  unsigned int dhcpTimoutCounter = 20;                // 20 s
  while ((dhcpTimoutCounter > 0) && !cc3000.checkDHCP())
  {
    dhcpTimoutCounter--;
    delay(dhcpConnectRetryInterval);
  }
  if (0 == dhcpTimoutCounter)
  {
    Serial.println(F("Failed!"));
    return; // bail out
  }

  /* Display the IP address DNS, Gateway, etc. */
  while (!displayConnectionDetails())
  {
    delay(1000);
  }
}

//-----------------------------------------------------------------------------

/**************************************************************************/
/*!
    @brief  Tries to read the CC3000's internal firmware patch ID
*/
/**************************************************************************/
uint16_t checkFirmwareVersion(void)
{
  uint8_t major, minor;
  uint16_t version;

#ifndef CC3000_TINY_DRIVER
  if(!cc3000.getFirmwareVersion(&major, &minor))
  {
    Serial.println(F("Unable to retrieve the firmware version!\r\n"));
    version = 0;
  }
  else
  {
    Serial.print(F("Firmware V. : "));
    Serial.print(major); Serial.print(F(".")); Serial.println(minor);
    version = major; version <<= 8; version |= minor;
  }
#endif
  return version;
}

/**************************************************************************/
/*!
    @brief  Tries to read the 6-byte MAC address of the CC3000 module
*/
/**************************************************************************/
void displayMACAddress(void)
{
  uint8_t macAddress[6];

  if(!cc3000.getMacAddress(macAddress))
  {
    Serial.println(F("Unable to retrieve MAC Address!\r\n"));
  }
  else
  {
    Serial.print(F("MAC Address : "));
    cc3000.printHex((byte*)&macAddress, 6);
  }
}

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
  uint8_t valid, rssi, sec;
  uint32_t index;
  char ssidname[33];

  if (!cc3000.startSSIDscan(&index))
  {
    Serial.println(F("SSID scan failed!"));
    return false;
  }

  Serial.print(F("Networks found: ")); Serial.println(index);
  Serial.println(F("================================================"));

  while (index > 0)
  {
    bool match = false;
    index--;
    valid = cc3000.getNextSSID(&rssi, &sec, ssidname);

    if (0 == strncmp(ssidname, searchSSID, sizeof(ssidname)))
    {
      found = true;
      match = true;
    }

    Serial.print(F("SSID Name    : ")); Serial.print(ssidname);
    if (match)
    {
      Serial.print(" <<found>>");
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
  processEchoServer();
}

//-----------------------------------------------------------------------------

void processEchoServer()
{
  // Try to get a client which is connected.
  Adafruit_CC3000_ClientRef client = echoServer.available();

  if (client)
  {
    // Check if there is data available to read.
    if (client.available() > 0)
    {
      // Read a byte and write it to all clients.
      uint8_t ch = client.read();
      if ('g' == ch)
      {
        Serial.println(F("processEchoServer(): g -> start"));
        if ((0 != cmdSeq) && (0 != ultrasonicSensorFront) && (0 != battery))
        {
          if (!cmdSeq->isRunning()
              && !ultrasonicSensorFront->isObstacleDetected()
              && !battery->isBattVoltageBelowStopThreshold()
              && !battery->isBattVoltageBelowShutdownThreshold())
          {
            cmdSeq->start();
          }
        }
      }
      else if ('h' == ch)
      {
        Serial.println(F("processEchoServer(): h -> stop"));
        if (0 != cmdSeq)
        {
          cmdSeq->stop();
        }
      }
      else if ('i' == ch)
      {
        client.print(F("Lintilla, Robot ID="));
        if (0 != ivm)
        {
          client.print(ivm->getDeviceId());
        }
        client.write('\r');
        client.write('\n');
      }
      else if ('l' == ch)
      {
        if (0 != mmi)
        {
          mmi->setBackLightOn(true);
        }
      }
      else if ('o' == ch)
      {
        if (0 != mmi)
        {
          mmi->setBackLightOn(false);
        }
      }
      client.write(ch);
      client.write('\r');
      client.write('\n');
    }
  }
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
  Serial.println("IVM:");
  Serial.println("---------------------------------------------------");
  Serial.print("ID: "); Serial.println(ivm->getDeviceId());
  Serial.print("IVM V."); Serial.println(ivm->getIvmVersion());
  Serial.print("BattVoltSenseFactor = "); Serial.println(ivm->getBattVoltageSenseFactor());
  Serial.println("---------------------------------------------------");
  batteryAdapter = new LintillaBatteryAdapter();
  battery = new Battery(batteryAdapter);
  batteryAdapter->attachBattery(battery);
  batteryAdapter->attachLintillaMmi(mmi);
  batteryAdapter->attachLintillaIvm(ivm);
  batteryAdapter->attachCmdSequence(cmdSeq);

  //---------------------------------------------------------------------------
  // Speed Sensors
  //---------------------------------------------------------------------------
  speedSensors = new SpeedSensors();

  //---------------------------------------------------------------------------
  // Motor Drivers and Speed Control
  //---------------------------------------------------------------------------
  traction = new Traction();

  //---------------------------------------------------------------------------
  // Command Sequence
  //---------------------------------------------------------------------------
  cmdSeq = new CmdSequence(new ACmdAdapter(traction));

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
  // Ultrasonic Ranging
  //---------------------------------------------------------------------------
//  ultrasonicSensorFront = new UltrasonicSensorHCSR04(triggerPin, echoPin);
//  ultrasonicSensorFront->attachAdapter(new AnUltrasonicSensorAdapter(cmdSeq));

  tractionAdapter = new ATractionAdapter(ultrasonicSensorFront);
  traction->attachAdapter(tractionAdapter);

  //---------------------------------------------------------------------------
  // MMI
  //---------------------------------------------------------------------------
  mmi = new LintillaMmi(new ALintillaMmiAdapter(battery, cmdSeq, ivm, ultrasonicSensorFront,
                                                &cc3000, speedSensors));

  //---------------------------------------------------------------------------
  // WiFi and Socket Server
  //---------------------------------------------------------------------------
  wifiReconnectTimer = new Timer(new WifiReconnectTimerAdapter(wifiReconnectTimer), Timer::IS_RECURRING, cWifiReconnectInterval);
  connectWiFi();
  startEchoServer();
}

//-----------------------------------------------------------------------------

void startEchoServer()
{
  if (cc3000.checkConnected())
  {
    // Start listening for connections
    echoServer.begin();
    Serial.println(F("Echo Server is listening for connections..."));
  }
}

//-----------------------------------------------------------------------------

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
      Serial.println("i received");
    }
    else if ('d' == inChar)
    {
      Serial.println("d received");
    }
  }
}
