/*
 * ProductDebug.cpp
 *
 *  Created on: 14.06.2016
 *      Author: nid
 */

#include "ProductDebug.h"

#include <Arduino.h>
#include <AppDebug.h>

#ifdef ESP8266
#include <ESP8266WiFi.h>
#endif
#include <Timer.h>
#include <SerialCommand.h>
#include <DbgCliNode.h>
#include <DbgCliTopic.h>
#include <DbgCliCommand.h>
#include <DbgTraceContext.h>
#include <DbgTracePort.h>
#include <DbgTraceLevel.h>
#include <DbgPrintConsole.h>
#include <DbgTraceOut.h>

//-----------------------------------------------------------------------------
// WiFi Commands
//-----------------------------------------------------------------------------
class DbgCli_Cmd_WifiMac : public DbgCli_Command
{
public:
  DbgCli_Cmd_WifiMac(DbgCli_Topic* wifiTopic)
  : DbgCli_Command(wifiTopic, "mac", "Print MAC Address.")
  { }

  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
  {
    Serial.print("Wifi MAC: ");
// TODO:   Serial.println(WiFi.macAddress().c_str());
  }
};

class DbgCli_Cmd_WifiStat : public DbgCli_Command
{
public:
  DbgCli_Cmd_WifiStat(DbgCli_Topic* wifiTopic)
  : DbgCli_Command(wifiTopic, "stat", "Show WiFi connection status.")
  { }

  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
  {
    // TODO
//    Serial.println(WiFi.getAutoConnect() ? "WiFi is autoconnecting" : "WiFi is not autoconnecting");
//    wl_status_t wlStatus = WiFi.status();
//    Serial.println(wlStatus == WL_NO_SHIELD       ? "NO_SHIELD      " :
//                   wlStatus == WL_IDLE_STATUS     ? "IDLE_STATUS    " :
//                   wlStatus == WL_NO_SSID_AVAIL   ? "NO_SSID_AVAIL  " :
//                   wlStatus == WL_SCAN_COMPLETED  ? "SCAN_COMPLETED " :
//                   wlStatus == WL_CONNECTED       ? "CONNECTED      " :
//                   wlStatus == WL_CONNECT_FAILED  ? "CONNECT_FAILED " :
//                   wlStatus == WL_CONNECTION_LOST ? "CONNECTION_LOST" :
//                   wlStatus == WL_DISCONNECTED    ? "DISCONNECTED   " : "UNKNOWN");
  }
};


class DbgCli_Cmd_WifiDis : public DbgCli_Command
{
public:
  DbgCli_Cmd_WifiDis(DbgCli_Topic* wifiTopic)
  : DbgCli_Command(wifiTopic, "dis", "Disconnect WiFi.")
  { }

  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
  {
// TODO:   WiFi.disconnect(false);
  }
};

class DbgCli_Cmd_WifiCon : public DbgCli_Command
{
public:
  DbgCli_Cmd_WifiCon(DbgCli_Topic* wifiTopic)
  : DbgCli_Command(wifiTopic, "con", "Connect to WiFi.")
  { }

  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
  {
    if (argc - idxToFirstArgToHandle != 2)
    {
      printUsage();
    }
    else
    {
// TODO:     WiFi.begin(args[idxToFirstArgToHandle], args[idxToFirstArgToHandle+1]);
    }
  }

  void printUsage()
  {
    Serial.println(getHelpText());
    Serial.println("Usage: dbg wifi con <SSID> <passwd>");
  }
};

////-----------------------------------------------------------------------------
//// ThingSpeak Commands
////-----------------------------------------------------------------------------
//class DbgCli_Cmd_ThingSpeakSetField : public DbgCli_Command
//{
//public:
//  DbgCli_Cmd_ThingSpeakSetField(DbgCli_Topic* thingSpeakTopic)
//  : DbgCli_Command(thingSpeakTopic, "set", "Set ThingSpeak field value.")
//  { }
//
//  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
//  {
//    if (argc - idxToFirstArgToHandle >= 1)
//    {
//      unsigned int field = 1;
//      if (argc - idxToFirstArgToHandle == 2)
//      {
//        field = atoi(args[idxToFirstArgToHandle+1]);
//      }
//      Serial.print("Field: ");
//      Serial.print(field);
//      Serial.print(" - Value: ");
//      Serial.println(static_cast<float>(atof(args[idxToFirstArgToHandle])), 2);
//      ThingSpeak.setField(field, static_cast<float>(atof(args[idxToFirstArgToHandle])));
//    }
//    else
//    {
//      printUsage();
//    }
//  }
//
//  void printUsage()
//  {
//    Serial.println(getHelpText());
//    Serial.println("Usage: dbg thgspk set <value> [field]");
//    Serial.println("       field: 1..8, default: 1");
//  }
//};
//
//unsigned long thingSpeakChannelId = 0;
//
//class DbgCli_Cmd_ThingSpeakChID : public DbgCli_Command
//{
//public:
//  DbgCli_Cmd_ThingSpeakChID(DbgCli_Topic* thingSpeakTopic)
//  : DbgCli_Command(thingSpeakTopic, "chid", "Set ThingSpeak Channel ID.")
//  { }
//
//  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
//  {
//    if (argc - idxToFirstArgToHandle != 1)
//    {
//      printUsage();
//    }
//    else
//    {
//      thingSpeakChannelId = atoi(args[idxToFirstArgToHandle]);
//      Serial.print("ThingSpeak write fields, Channel Id: ");
//      Serial.println(thingSpeakChannelId);
//    }
//  }
//
//  void printUsage()
//  {
//    Serial.println(getHelpText());
//    Serial.println("Usage: dbg thgspk chid <channelId>");
//  }
//};
//
//char thingSpeakAPIKey[16+1];
//
//class DbgCli_Cmd_ThingSpeakAPIKey : public DbgCli_Command
//{
//public:
//  DbgCli_Cmd_ThingSpeakAPIKey(DbgCli_Topic* thingSpeakTopic)
//  : DbgCli_Command(thingSpeakTopic, "key", "Set ThingSpeak API key.")
//  { }
//
//  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
//  {
//    if (argc - idxToFirstArgToHandle != 1)
//    {
//      printUsage();
//    }
//    else
//    {
//      strncpy(thingSpeakAPIKey, args[idxToFirstArgToHandle], 16);
//      Serial.print("ThingSpeak write fields, API Key: ");
//      Serial.println(thingSpeakAPIKey);
//    }
//  }
//
//  void printUsage()
//  {
//    Serial.println(getHelpText());
//    Serial.println("Usage: dbg thgspk key <APIKey>");
//  }
//};

//class DbgCli_Cmd_ThingSpeakWriteFields : public DbgCli_Command
//{
//public:
//  DbgCli_Cmd_ThingSpeakWriteFields(DbgCli_Topic* thingSpeakTopic)
//  : DbgCli_Command(thingSpeakTopic, "wr", "Write ThingSpeak fields.")
//  { }
//
//  void execute(unsigned int argc, const char** args, unsigned int idxToFirstArgToHandle)
//  {
//    if (argc - idxToFirstArgToHandle != 0)
//    {
//      printUsage();
//    }
//    else
//    {
//      Serial.print("ThingSpeak write fields, Channel Id: ");
//      Serial.print(thingSpeakChannelId);
//      Serial.print(", API Key: ");
//      Serial.println(thingSpeakAPIKey);
//      int status = ThingSpeak.writeFields(thingSpeakChannelId, thingSpeakAPIKey);
//      Serial.print("ThingSpeak write done, result: ");
//      Serial.println(status ==  200 ? "OK / Success                                                                           " :
//                     status ==  404 ? "Incorrect API key (or invalid ThingSpeak server address)                               " :
//                     status == -101 ? "Value is out of range or string is too long (> 255 characters)                         " :
//                     status == -201 ? "Invalid field number specified                                                         " :
//                     status == -210 ? "setField() was not called before writeFields()                                         " :
//                     status == -301 ? "Failed to connect to ThingSpeak                                                        " :
//                     status == -302 ? "Unexpected failure during write to ThingSpeak                                          " :
//                     status == -303 ? "Unable to parse response                                                               " :
//                     status == -304 ? "Timeout waiting for server to respond                                                  " :
//                     status == -401 ? "Point was not inserted (most probable cause is the rate limit of once every 15 seconds)" : "UNKNOWN");
//    }
//  }
//
//  void printUsage()
//  {
//    Serial.println(getHelpText());
//    Serial.println("Usage: dbg thgspk wr");
//  }
//};

//-----------------------------------------------------------------------------

void setupProdDebugEnv()
{
  setupDebugEnv();

  Serial.println();
  Serial.println("---------------------------------------------");
  Serial.println("Hello from Lintilla!");
  Serial.println("---------------------------------------------");
  Serial.println();

  //-----------------------------------------------------------------------------
  // WiFi Commands
  //-----------------------------------------------------------------------------
#ifdef ESP8266
  DbgCli_Topic* wifiTopic = new DbgCli_Topic(DbgCli_Node::RootNode(), "wifi", "WiFi debug commands");
  new DbgCli_Cmd_WifiMac(wifiTopic);
  new DbgCli_Cmd_WifiStat(wifiTopic);
  new DbgCli_Cmd_WifiDis(wifiTopic);
  new DbgCli_Cmd_WifiCon(wifiTopic);
#endif

  //-----------------------------------------------------------------------------
  // ThingSpeak Commands
  //-----------------------------------------------------------------------------
#ifdef ESP8266
  DbgCli_Topic* thingSpeakTopic = new DbgCli_Topic(DbgCli_Node::RootNode(), "thgspk", "ThingSpeak debug commands");
  new DbgCli_Cmd_ThingSpeakSetField(thingSpeakTopic);
  new DbgCli_Cmd_ThingSpeakChID(thingSpeakTopic);
  new DbgCli_Cmd_ThingSpeakAPIKey(thingSpeakTopic);
  new DbgCli_Cmd_ThingSpeakWriteFields(thingSpeakTopic);
#endif

}
