
#include <ESP8266WiFi.h>
#include <multi_channel_relay.h>
#include <Wire.h>
#include "wifi.h"

#define MAX_RESPONSE_TIME 10000 // millis to wait for a response
#define MAX_MESSAGE_SIZE 1000
#define MESSAGE_CHECK_INTERVAL 10000UL // 10 secs
#define NETWORK_CHECK_INTERVAL 30000UL // Check network up every 30secs when down
#define BUFF_SIZE MAX_MESSAGE_SIZE + 6
uint8_t buff[BUFF_SIZE];
#define MAX_GET_MSG_SIZE 64    // Max size of dynamic get msg with params
#define COMMAND_HEADER_LEN 3
#define CMD_BUFF_SIZE MAX_GET_MSG_SIZE + COMMAND_HEADER_LEN
char getMessageBuff[MAX_GET_MSG_SIZE];

String GET_STR = "GET /powerc?s=20&r=";
String HTTP_STR = " HTTP/0.9";
int SDA_PIN = 2;
int SCL_PIN = 0;
int RELAY_ADDR = 0x11;

char CONTENT_LENGTH[] = "Content-Length: ";
char HTTP_09[] = "HTTP/0.9 ";
byte mac[6];

struct PowerControlMsg
{
  int8_t relay;
  int8_t state;
};

// Initialize the client library
WiFiClient client;
// Initialise Relay
Multi_Channel_Relay relay;

bool networkUp = false;
unsigned long currentMillis = 0;
unsigned long networkUpTime = 0; // Time at which the network was last up
unsigned long lastNetworkCheckTime = 0;

void setup() {
  Serial.begin(9600);
  while(!Serial);   

  Serial.println("Initialising Wire library for I2C");
  Wire.begin(SDA_PIN, SCL_PIN);
  // Wire.begin();

  // Set Relay I2C address and start relay
  Serial.println("Initialising Relay library");
  relay.begin(RELAY_ADDR);
  /* Read firmware  version */
  Serial.print("Relay firmware version: ");
  Serial.print("0x");
  Serial.print(relay.getFirmwareVersion(), HEX);
  Serial.println();
  
  // listNetworks();
  Serial.println("Config Wifi:");
  WiFi.config(clientIP, dns, gateway);
  Serial.println("Starting Wifi:");
  WiFi.begin(SSID_NAME, PASSWORD);
  printWifiStatus();
  networkUp = true;
  Serial.println("Entering loop");
}


void loop() {
  currentMillis = millis();
  uint8_t relayStatus = relay.getChannelState();
  Serial.print("Relay Status: ");
  Serial.println(relayStatus);
  // Check for any messages
  if (networkUp)
  {
    uint16_t respLen = getMessage(relayStatus);
    if (respLen > 0) {
      processMessage();
    }
  }

  if (!networkUp && (currentMillis - lastNetworkCheckTime) > NETWORK_CHECK_INTERVAL)
  {
    lastNetworkCheckTime = currentMillis;
    uint8_t stat = checkNetworkUp();
    if (stat == 0)
    {
      networkUp = true;
      networkUpTime = currentMillis;
    }
  }
  delay(MESSAGE_CHECK_INTERVAL);
}

uint16_t getMessage(uint8_t relayStatus) {
  uint16_t respLen = 0;
  if (checkNetworkUp()) {
    if (openTCP()) {
      drainWifi();
      Serial.println("Sending HTTP Get"); //Zero first byte == success
      sendHttpRequest(relayStatus);
      respLen = waitForHttpResponse();
      if (respLen > 0) {
        Serial.print("Rx message: Length ");
        Serial.println(respLen);
        printBuff(respLen);
      } 
       else {
         Serial.println("No relay command received");
       }
    } else {
      // Server down error message
      Serial.print("2 Server ");
      Serial.print(serverAddr[0]);
      Serial.print(".");
      Serial.print(serverAddr[1]);
      Serial.print(".");
      Serial.print(serverAddr[2]);
      Serial.print(".");
      Serial.print(serverAddr[3]);
      Serial.print(":");
      Serial.print(PORT);
      Serial.print(" down");
      Serial.println("EOL");
    }
  } else {
    // Wifi error
    Serial.print("1 Wifi Down: ");
    Serial.print(getWifiStatus());
    Serial.println();
  }
  return respLen;
}

void processMessage() {

  PowerControlMsg *pc = (PowerControlMsg *)&buff[4]; // Start of content
  if ((pc->state == 1 || pc->state == 0) && (pc->relay >= 0 && pc->relay <= 3)) {
    //Valid command
    if (pc->state == 1) {
      relay.turn_on_channel(pc->relay);
    } else {
      relay.turn_off_channel(pc->relay);
    }
  } else {
    //Invalid
    Serial.print("Invalid command received: Relay no: ");
    Serial.print(pc->relay);
    Serial.print(" State: ");
    Serial.println(pc->state);
  }
}

String getWifiStatus() {
  String retStatus = "??";
  int stat = WiFi.status();
  switch (stat) {
    case WL_IDLE_STATUS: 
      retStatus = "IDLE";
      break;
    case WL_NO_SSID_AVAIL:
      retStatus = "NO AP AVAIL";
      break;
    case WL_CONNECT_FAILED:
      retStatus = "CONNECT FAILED";
      break;
    case WL_WRONG_PASSWORD:
      retStatus = "WRONG PWD";
      break;
    case WL_DISCONNECTED:
      retStatus = "DISCONNECTED";
      break;
    case WL_CONNECTED:
      retStatus = "CONNECTED";
      break;
    default:
      retStatus = String(stat);
  }
  return retStatus;
}

void printWifiStatus() {
    Serial.print(WiFi.SSID());
    IPAddress ip = WiFi.localIP();
    Serial.print(" IP:");
    Serial.print(ip);
    WiFi.macAddress(mac);
    Serial.print(" MAC: ");
    Serial.print(mac[0],HEX);
    Serial.print(":");
    Serial.print(mac[1],HEX);
    Serial.print(":");
    Serial.print(mac[2],HEX);
    Serial.print(":");
    Serial.print(mac[3],HEX);
    Serial.print(":");
    Serial.print(mac[4],HEX);
    Serial.print(":");
    Serial.print(mac[5],HEX);  
}

bool checkNetworkUp()
{
  // Check network up
  return WiFi.status() == WL_CONNECTED;
}

bool openTCP()
{
  bool connected = false;
  //  debugSerial.println("Open TCP ");
  if (client.connect(serverAddr, PORT))
  {
    //      Serial.println("connected");
    connected = true;
  }
  return connected;
}

void sendHttpRequest(uint8_t relayStatus)
{
  // Make a HTTP request
//  Serial.print("Sending get:");
//  for (int i = 0; i<MAX_GET_MSG_SIZE; i++) {
//    getMessageBuff[i] = 0;
//  }
//  snprintf(getMessageBuff, MAX_GET_MSG_SIZE, GET_MESSAGE_TEMPLATE, &commandBuff[COMMAND_HEADER_LEN] );
//  String msg = String((char *)getMessageBuff);
//  Serial.println(msg);
//  Serial.print("Len: ");
//  Serial.println(msg.length());
//  Serial.println((char *)getMessageBuff);
//  uint8_t bytesWritten = client.println("GET /motd HTTP/0.9");
//  uint8_t bytesWritten = client.println((char *)getMessageBuff);
//  for (int i=0; i<MAX_GET_MSG_SIZE ; i++) {
//    client.write(getMessageBuff[i]);
//    if ((uint8_t)getMessageBuff[i] == 0) {
//      break;
//    }
//  }
//  client.println("GET /motd HTTP/0.9");
  //Wifi library wants a String
  String msg = GET_STR + relayStatus + HTTP_STR;
  client.println(msg);
  client.println();
  client.println();
  Serial.println(msg);
//  for (int i=0; i<MAX_GET_MSG_SIZE; i++) {
//    Serial.print(getMessageBuff[i]);
//    if ((uint8_t)getMessageBuff[i] == 0) {
//      break;
//    }
//  }
//  Serial.println();
}

uint16_t waitForHttpResponse() {
  unsigned long start = millis();
  uint16_t bufPos = 0;
  uint16_t contentLen = 0;
  unsigned long waitTime = 0;
  bool gotRespCode = false;
  bool gotMsg = false;
  while (waitTime < MAX_RESPONSE_TIME && !gotMsg)
  {
    uint16_t len = receiveClientData(bufPos);
    if (len > 0) { 
      bufPos += len;
      if (!gotRespCode) {
        uint16_t pos = findStringInBuff(buff, HTTP_09, sizeof(HTTP_09) - 1, bufPos);
        if (pos != -1 && (pos+3) < bufPos) {
          pos += sizeof(HTTP_09)-1;
          // Retrieve respCode
          char respCode[] = "   ";
          uint16_t endPos = pos;
          for (; endPos < (pos + 3); endPos++) {
            respCode[endPos - pos] = buff[endPos];
          }
          respCode[endPos - pos] = '\0'; // null terminate
          Serial.print("Resp Code:");
          Serial.print((uint8_t)respCode[0], HEX);
          Serial.print((uint8_t)respCode[1], HEX);
          Serial.println((uint8_t)respCode[2], HEX);
          uint16_t resp = atoi(&respCode[0]);
          if (resp != 200) {
            Serial.print("1Server Error Code: ");
            Serial.print(resp);
            Serial.println("EOL");
            drainWifi();
            break;
          }
          gotRespCode = true;
        }
      } 
      if (gotRespCode && contentLen == 0) {
        // Look for Content length in header part of response
        uint16_t pos = findStringInBuff(buff, CONTENT_LENGTH, sizeof(CONTENT_LENGTH) - 1, bufPos);
        if (pos != -1 && (pos+4) < bufPos) {
          pos += sizeof(CONTENT_LENGTH)-1;
          // Retrieve content length
          char lenStr[] = "    ";
          uint16_t endPos = pos;
          for (; endPos < (pos + 3); endPos++) {
            if (int(buff[endPos]) == 13) {
              endPos++;
              break; // Reached end of line
            }
            lenStr[endPos - pos] = buff[endPos];
          }
          lenStr[endPos - pos] = '\0'; // null terminate
          Serial.print("Content Len Str:");
          Serial.print((uint8_t)lenStr[0], HEX);
          Serial.print((uint8_t)lenStr[1], HEX);
          Serial.print((uint8_t)lenStr[2], HEX);
            contentLen = atoi(&lenStr[0]);
          Serial.print("Content Len:");
          Serial.println(contentLen);
          if (contentLen > 0 && bufPos > endPos+3) {
            endPos += 3; // 0xd, 0xa, 0xd, 0xa at end of content str and before content
            // shift bytes down in buff
            shiftBuffDown(buff, MAX_MESSAGE_SIZE, endPos, bufPos); // Now we should just have the message content in the buff..
          } else {
            contentLen = 0;
          }
        }
      }
    }
    if (contentLen > 0 && bufPos >= contentLen) {
      // got all of the message
      gotMsg = true;
    }
    if (!gotMsg)
    {
      // Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - start;
  }
  if (waitTime >= MAX_RESPONSE_TIME && !gotMsg)
  {
    Serial.print("1Msg Receive timeout:");
    Serial.print(" Rx bytes:");
    Serial.print(bufPos);
    Serial.print(" contentLen:");
    Serial.print(contentLen);
    Serial.println("EOL");
    printBuff(bufPos);
  }
  return contentLen;
}

uint16_t findStringInBuff(uint8_t bf[], char str[], uint8_t strLen, uint16_t maxPos) {
//  debugSerial.print("Looking for ");
//  debugSerial.print(str);
//  debugSerial.print(" Size:");
//  debugSerial.print(strLen);
//  debugSerial.print(" MaxPos:");
//  debugSerial.println(maxPos);
//  printBuff(maxPos);
  uint16_t startPos = -1;
  for (int i = 0; (i+strLen)<maxPos; i++) {
    if (bf[i] == str[0]) {
      startPos = i;
      uint16_t strPos = 0;
      uint16_t endStr = i+strLen;
      int j = i;
      for (;j < endStr && j<maxPos; j++) {
        if (bf[j] != str[strPos++]) {
          break;
        }
      }
      if (j >= endStr) { //All chars matched
        break;
      } else {
        startPos = -1;
      }
    }
  }
  return startPos;
}

uint16_t receiveClientData(uint16_t startPosition) {
  uint16_t len = 0;
  uint16_t pos = startPosition;
  while (client.available() > 0 && pos < BUFF_SIZE) {
    uint8_t b = client.read();
    buff[pos++] = b;
    len++;
    // debugSerial.print(b, HEX);
  }
  return len;
}

void shiftBuffDown(uint8_t bf[], uint16_t buffSize, uint16_t pos, uint16_t maxPos)
{
  //  debugSerial.print("Downshift by ");
  //  debugSerial.println(pos);
  //  printBuff(maxPos);
  uint8_t p = pos;
  for (int i = 0; i < maxPos; i++)
  {
    if (p < buffSize) {
      bf[i] = bf[p++];
    } else {
      bf[i] = 0;
    }
  }
  for (int i=maxPos; i<buffSize; i++) {
      bf[i] = 0;
  }
  //  debugSerial.println("After:");
  //  printBuff(maxPos);
}

void drainWifi()
{
  delay(10);
  int total = 0;
  while (uint16_t len = receiveClientData(0) > 0)
  {
    total += len;
    delay(10);
  }
  //  if (total > 0) {
  //    debugSerial.print("Drained: ");
  //    debugSerial.println(total);
  //  }
  resetBuffer();
}

void resetBuffer()
{
  // Reset receive buffer
  for (uint32_t i = 0; i < MAX_MESSAGE_SIZE; i++)
  {
    buff[i] = 0;
  }
}

void printBuff(uint32_t pos) {
    Serial.print("Hex:");
    for (uint32_t i = 0; i<pos && i<MAX_MESSAGE_SIZE; i++) {
      Serial.print((uint8_t)buff[i], HEX);
      Serial.print(',');
    }
    Serial.print("\nText:");
    for (uint32_t i = 0; i<pos && i<MAX_MESSAGE_SIZE; i++) {
      Serial.print((char) buff[i]);
    }
    Serial.println();
}

void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true);
  }

  // print the list of networks seen:
  Serial.print("number of available networks:");
  Serial.println(numSsid);

  // print the network number and name for each network found:
  for (int thisNet = 0; thisNet < numSsid; thisNet++) {
    Serial.print(thisNet);
    Serial.print(") ");
    Serial.print(WiFi.SSID(thisNet));
    Serial.print("\tSignal: ");
    Serial.print(WiFi.RSSI(thisNet));
    Serial.print(" dBm");
    Serial.println();
  }
}