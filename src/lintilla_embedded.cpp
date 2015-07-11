// Do not remove the include below
#include "lintilla_embedded.h"

#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <CmdHandler.h>
#include <SPI.h>
#include <string.h>
//#include "utility/debug.h"
//#include "CC3000_MDNS.h"
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
#include "DistanceCount.h"
#include "Battery.h"
#include "LintillaBatteryAdapter.h"
#include "aREST.h"
#include "RamUtils.h"
#include "Wire.h"
#include "FreeSixIMU.h"
#include "Cmd.h"
#include "PID_v1.h"
#include "DbgCliNode.h"
#include "DbgCliTopic.h"
#include "DbgCliCommand.h"
#include "DbgTracePort.h"
#include "DbgTraceContext.h"
#include "DbgTraceOut.h"
#include "DbgPrintConsole.h"
#include "DbgTraceLevel.h"

#include "LintillaMmiScreenFsm.h"

#define DEBUG_RAM 1  //Printing free Ram space with serial monitor
#define USE_WIFI 0
#define USE_HARD_CODED_WIFI_CREDENTIALS 1
#if USE_HARD_CODED_WIFI_CREDENTIALS
#define WLAN_SSID "LintillaNet"
#define WLAN_PASS "AnswerIs42"
#endif

//-----------------------------------------------------------------------------

void(* resetFunc) (void) = 0; //declare reset function at address 0 => will 'use' NULL pointer exception and then restart.

//-----------------------------------------------------------------------------
// Debugging
//-----------------------------------------------------------------------------

class DbgCli_Command_FreeRam : public DbgCli_Command
{
public:
  DbgCli_Command_FreeRam()
  : DbgCli_Command(DbgCli_Node::RootNode(), "ram", "Show free RAM space.")
  { }

  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
  {
    Serial.print(F("Free RAM: ")); Serial.print(RamUtils::getFreeRam(), DEC); Serial.println(" [bytes]");
  }
};

void dbgCliExecute(int arg_cnt, char** args);
void hello(int arg_cnt, char** args);

DbgTrace_Port* ramTestPort;

Timer* ramDebugTimer = 0;
const unsigned int c_ramDbgInterval = 5000;
class RamDebugTimerAdapter : public TimerAdapter
{
private:
//  DbgTrace_Port* m_trPort;

public:
  RamDebugTimerAdapter() { }
//  : m_trPort(new DbgTrace_Port("dbg/ram"))
//  {
//    if (0 != m_trPort)
//    {
//      m_trPort->setLevel(DbgTrace_Level::debug);
//    }
//  }

private:
  void timeExpired()
  {
    TR_PRINT_LONG(ramTestPort, DbgTrace_Level::debug, RamUtils::getFreeRam());
  }
};

//---------------------------------------------------------------------------
// Inventory Management
//---------------------------------------------------------------------------
LintillaIvm* ivm = 0;

bool isLintillaHw()
{
  typedef enum
  {
    LintillaIdMin = 0,
    LintillaId01  = 1,
    LintillaId02  = 2,
    LintillaId03  = 3,
    LintillaId04  = 4,
    LintillaId05  = 5,
    LintillaIdMax = 6
  } LintillaIds;
  bool isLHw = false;
  unsigned char deviceId = 0;
  if (0 != ivm)
  {
    deviceId = ivm->getDeviceId();
  }
  if ((LintillaIdMin < deviceId) && (deviceId < LintillaIdMax))
  {
    isLHw = true;
  }
  return isLHw;
}

//-----------------------------------------------------------------------------
// Battery Voltage Surveillance
//-----------------------------------------------------------------------------
Battery* battery = 0;
LintillaBatteryAdapter* batteryAdapter = 0;

//---------------------------------------------------------------------------
// Ultrasonic Ranging
//---------------------------------------------------------------------------
UltrasonicSensor* ultrasonicSensorFront = 0;

//---------------------------------------------------------------------------
// FreeSixIMU
//---------------------------------------------------------------------------
FreeSixIMU* my6IMU = 0;

//---------------------------------------------------------------------------
// PID controllers for each wheel
//---------------------------------------------------------------------------
PID* pidLeftWheel = 0;
PID* pidRightWheel = 0;

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
// Test Sequence
//---------------------------------------------------------------------------
CmdSequence* testCmdSeq = 0;

//---------------------------------------------------------------------------
// REST Server
//---------------------------------------------------------------------------
// Create aREST instance
aREST rest = aREST();

void processRestServer();
void setupRestServer();
void startRestServer();

// REST API function declarations
int lcdBacklightControl(String command);  // light
int test(String command); // test
int stop(String command); // stop
int move(String command); // move

//---------------------------------------------------------------------------
// MDNS responder
//---------------------------------------------------------------------------
//MDNSResponder* mdns = 0;

//---------------------------------------------------------------------------
// WiFi Driver
//---------------------------------------------------------------------------
// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  12 // WEN, was on  5 before
#define ADAFRUIT_CC3000_CS    13 // WCS, was on 10 before
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIV2); // you can change this clock speed but DI


// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define LISTEN_PORT     80    // What TCP port to listen on for connections.  The echo protocol uses port 7.

uint32_t currentIpAddress = 0;
Adafruit_CC3000_Server restServer(LISTEN_PORT);

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
    if (!cc3000.checkConnected() && isLintillaHw())
    {
      Serial.print("Lintilla lost WiFi connection, reconnecting!!\n");
      delay(2000);
      cc3000.reboot();
      connectWiFi();
      startRestServer();
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
  bool bailOut = false;

  Serial.println(F("\nconnectWifi(): using CC3000 driver!"));
  Serial.print("Free RAM: "); Serial.println(RamUtils::getFreeRam(), DEC);

  /* Initialize the module */
  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Unable to initialise the CC3000! Check your wiring?"));
    bailOut = true; // bail out
  }

  if (!bailOut)
  {
    uint16_t firmware = checkFirmwareVersion();
    if (firmware < 0x113) {
      Serial.println(F("Wrong CC3000 firmware version!"));
      bailOut = true;  // bail out
    }
  }

#if USE_HARD_CODED_WIFI_CREDENTIALS
  const char* ssid = WLAN_SSID;
#else
  char ssid[wlan_max_length];
  if (0 != ivm)
  {
    ivm->getWlanSSID(ssid);
  }
#endif
  if (!bailOut)
  {
    displayMACAddress();

    /* Attempt to connect to an access point */
    Serial.print(F("\nAttempting to connect to ")); Serial.println(ssid);

    if (!isSSIDPresent(ssid))
    {
      Serial.print(F("Failed! SSID "));
      Serial.print(ssid);
      Serial.println(F(" not found!"));
      bailOut = true;  // bail out
    }
  }

  if (!bailOut)
  {
    /* Delete any old connection data on the module */
    Serial.println(F("\nDeleting old connection profiles"));
    if (!cc3000.deleteProfiles()) {
      Serial.println(F("Failed!"));
      bailOut = true;  // bail out
    }
  }

#if USE_HARD_CODED_WIFI_CREDENTIALS
  const char* pass = WLAN_PASS;
#else
  char pass[wlan_max_length];
  if (0 != ivm)
  {
    ivm->getWlanPASS(pass);
  }
#endif

  if (!bailOut)
  {
    /* Attempt to connect to an access point */
    if (!cc3000.connectToAP(ssid, pass, WLAN_SECURITY))
    {
      // time out after 10 s
      Serial.println(F("Failed!"));
      bailOut = true;  // bail out
    }
  }

  if (!bailOut)
  {
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
      bailOut = true;  // bail out
    }
  }

  if (!bailOut)
  {
    /* Display the IP address DNS, Gateway, etc. */
    while (!displayConnectionDetails())
    {
      delay(1000);
    }
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

//  // Handle any multicast DNS requests
//  if (0 != mdns)
//  {
//    mdns->update();
//  }
  cmdPoll();
#if USE_WIFI
  if (isLintillaHw())
  {
    processRestServer();
  }
#endif
}

//-----------------------------------------------------------------------------

//The setup function is called once at startup of the sketch
void setup()
{
  cmdInit(115200); //contains Serial.begin(115200);

  Serial.println(F("\nHello from Lintilla!"));

  //---------------------------------------------------------------------------
  // Debug Cli
  //---------------------------------------------------------------------------
  DbgCli_Node::AssignRootNode(new DbgCli_Topic(0, "dbg", "Lintilla Debug CLI Root Node."));
  new DbgCli_Command_FreeRam();
  // adding CLI Commands
  cmdAdd("hello", hello);
  cmdAdd("dbg", dbgCliExecute);

  //---------------------------------------------------------------------------
  // Debug Trace
  //---------------------------------------------------------------------------

  DbgCli_Topic* traceTopic = new DbgCli_Topic(DbgCli_Node::RootNode(), "tr", "Modify debug trace");
  DbgTrace_Context* traceContext = new DbgTrace_Context(traceTopic);
  new DbgTrace_Out(DbgTrace_Context::getContext(), "traceConsoleOut", new DbgPrint_Console());

  ramTestPort = new DbgTrace_Port(DbgTrace_Context::getContext(), "ram", DbgTrace_Context::getContext()->getTraceOut("traceConsoleOut"), DbgTrace_Level::debug);

#if DEBUG_RAM
  // Print free RAM periodically
  Serial.print("Free RAM: "); Serial.print(RamUtils::getFreeRam(), DEC); Serial.println(" [bytes]");
  ramDebugTimer = new Timer(new RamDebugTimerAdapter(), Timer::IS_RECURRING, c_ramDbgInterval);
#endif
  //---------------------------------------------------------------------------
  // Inventory Management
  //---------------------------------------------------------------------------
//  #include "EEPROM.h"
//  EEPROM.write(1, 2); // hack to set IVM version back to one before, triggers eval. of all items in LintillaIvm::maintainVersionChange()
  ivm = new LintillaIvm();

  Serial.println("IVM:");
  Serial.println("---------------------------------------------------");
  Serial.print("ID: "); Serial.println(ivm->getDeviceId());
  Serial.print("IVM V."); Serial.println(ivm->getIvmVersion());
  Serial.print("BattVoltSenseFactor = "); Serial.println(ivm->getBattVoltageSenseFactor());
  Serial.println("---------------------------------------------------");
  if (isLintillaHw())
  {
    //---------------------------------------------------------------------------
    // Battery Voltage Surveillance
    //---------------------------------------------------------------------------
    batteryAdapter = new LintillaBatteryAdapter();
    battery = new Battery(batteryAdapter);
    batteryAdapter->attachBattery(battery);
    batteryAdapter->attachLintillaMmi(mmi);
    batteryAdapter->attachLintillaIvm(ivm);
    batteryAdapter->attachCmdSequence(testCmdSeq);
  }

  //---------------------------------------------------------------------------
  // Speed Sensors
  //---------------------------------------------------------------------------
  // speedSensors = new SpeedSensors();

  //---------------------------------------------------------------------------
  // Motor Drivers and Speed Control
  //---------------------------------------------------------------------------
  traction = new Traction();

  //---------------------------------------------------------------------------
  // Test Command Sequence
  //---------------------------------------------------------------------------
  testCmdSeq = new CmdSequence(new ACmdAdapter(traction));

  const int cSpinTime     =  300;
  const int cFwdTime      =  300;
  const int cInterDelay   =  500;
  const int cCtrlFwd      = 6000;

//  for (int i = 0; i <= 3; i++)
//  {
//    new CmdMoveForward(testCmdSeq, cFwdTime);
//    new CmdStop(testCmdSeq, cInterDelay);
//    new CmdSpinOnPlaceRight(testCmdSeq, cSpinTime);
//    new CmdStop(testCmdSeq, cInterDelay);
//  }
  new CmdMoveControlledForward(testCmdSeq, cCtrlFwd);
  new CmdStop(testCmdSeq, cInterDelay);


  CmdHandler* cmd = testCmdSeq->getFirstCmd();
  while (0 != cmd)
  {
    Serial.print("testCmdSeq: ");
    Serial.print(cmd->getName());
    Serial.print("; t=");
    Serial.print(cmd->getTime());
    Serial.println("[ms]");
    cmd = testCmdSeq->getNextCmd();
  }

  //---------------------------------------------------------------------------
  // Ultrasonic Ranging
  //---------------------------------------------------------------------------
  ultrasonicSensorFront = new UltrasonicSensorHCSR04();
  ultrasonicSensorFront->attachAdapter(new AnUltrasonicSensorAdapter(testCmdSeq));

  //---------------------------------------------------------------------------
  // Yaw controller with FreeSixIMU
  //---------------------------------------------------------------------------
  my6IMU = new FreeSixIMU();

  Wire.begin();
  delay(5);
  my6IMU->init();
  delay(20);

  tractionAdapter = new ATractionAdapter(ultrasonicSensorFront, my6IMU);
  traction->attachAdapter(tractionAdapter);

  //---------------------------------------------------------------------------
  // MMI
  //---------------------------------------------------------------------------
  mmi = new LintillaMmi(new ALintillaMmiAdapter(battery, testCmdSeq, ivm, ultrasonicSensorFront,
                                                &cc3000, speedSensors));

  //---------------------------------------------------------------------------
  // WiFi
  //---------------------------------------------------------------------------
#if USE_WIFI
  if (isLintillaHw())
  {
    wifiReconnectTimer = new Timer(new WifiReconnectTimerAdapter(wifiReconnectTimer), Timer::IS_RECURRING, cWifiReconnectInterval);
    connectWiFi();
  }
#endif

//  //---------------------------------------------------------------------------
//  // MDNSResponder
//  //---------------------------------------------------------------------------
//  mdns = new MDNSResponder();
//  char mdnsName[32];
//  char idString[4];
//  itoa(ivm->getDeviceId(), idString, 10);
//  strncat(mdnsName, "lintilla", 32);
//  strncat(mdnsName, idString, 32);
//
//  Serial.print(F("MDNS name is: "));
//  Serial.println(mdnsName);
//
//  // Start multicast DNS responder
//  if ((0 != mdns) && !mdns->begin(mdnsName, cc3000))
//  {
//    Serial.println(F("Error setting up MDNS responder!"));
//  }


  //---------------------------------------------------------------------------
  // REST Server
  //---------------------------------------------------------------------------
  startRestServer();
}

//-----------------------------------------------------------------------------

void processRestServer()
{
  if (cc3000.checkConnected())
  {
    // Handle REST calls
    Adafruit_CC3000_ClientRef client = restServer.available();
    rest.handle(client);
  }
}

void setupRestServer()
{
  // REST API function bindings
  rest.function("light", lcdBacklightControl);
  rest.function("test", test);
  rest.function("stop", stop);
  rest.function("move", move);

  // Give name and ID to device
  rest.set_name("Lintilla");
//  if (0 != ivm)
//  {
//    restId+=ivm->getDeviceId();
//    rest.set_id(const_cast<char*>(restId.c_str()));
//  }
//  else
//  {
    rest.set_id("000");
//  }
}

void startRestServer()
{
  if (cc3000.checkConnected())
  {
    setupRestServer();

    // Start REST server
    restServer.begin();
    Serial.println(F("REST server listening for connections..."));
  }
}

//-----------------------------------------------------------------------------
// REST API functions
//-----------------------------------------------------------------------------

// Custom functions accessible by the API
int lcdBacklightControl(String command)
{
  // Get state from command
  int state = command.toInt();
  if (0 != mmi)
  {
    mmi->setBackLightOn(state);
  }
  return 1;
}

int test(String command)
{
  if (0 != testCmdSeq)
  {
    testCmdSeq->start();
  }
  return 1;
}

int stop(String command)
{
  if (0 != testCmdSeq)
  {
    testCmdSeq->stop();
  }
  return 1;
}

int move(String command)
{
  int retVal = 1;

  Serial.print(F("move, parameters: "));
  Serial.println(command);

  bool isProtocolOk = true;
  bool isStraight = false;
  bool isForward = false;
  bool isRight = false;
  unsigned long timeMillis = 0;

  if (command.startsWith("f"))
  {
    isStraight = true;
    isForward = true;
    Serial.print(F("straight forward"));
  }
  else if (command.startsWith("b"))
  {
    isStraight = true;
    Serial.print(F("straight backward"));
  }
  else if (command.startsWith("l"))
  {
    Serial.print(F("spin on place left"));
  }
  else if (command.startsWith("r"))
  {
    isRight = true;
    Serial.print(F("spin on place right"));
  }
  else
  {
    isProtocolOk = false;
  }

  if (isProtocolOk)
  {
    timeMillis = static_cast<unsigned long>(command.substring(2).toInt());
    Serial.print(F(", t="));
    Serial.print(timeMillis);
    Serial.println(F("[ms]"));

    if (0 != traction)
    {
      if (isStraight)
      {
        traction->moveStraight(isForward);
      }
      else
      {
        traction->spinOnPlace(isRight, 0.0);
      }
    }
  }
  else
  {
    Serial.println(F("move: protocol error!"));
    retVal = 0;
  }

  return retVal;
}

//-----------------------------------------------------------------------------
// Arduino Cmd I/F
//-----------------------------------------------------------------------------
void hello(int arg_cnt, char **args)
{
  Serial.println("Hello world.");
}

void dbgCliExecute(int arg_cnt, char **args)
{
#if 0
  Serial.print("dbgCliExecute, arg_cnt=");
  Serial.print(arg_cnt);
  for (int i = 0; i < arg_cnt; i++)
  {
    Serial.print(", args[");
    Serial.print(i);
    Serial.print("]=");
    Serial.print(args[i]);
  }
  Serial.println("");
#endif
  if (0 != DbgCli_Node::RootNode())
  {
    const unsigned int firstArgToHandle = 1;
    DbgCli_Node::RootNode()->execute(static_cast<unsigned int>(arg_cnt), const_cast<const char**>(args), firstArgToHandle);
  }
}
