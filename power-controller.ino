
#include <ESP8266WiFi.h>
#include <multi_channel_relay.h>
#include <Wire.h>
#include "wifi.h"

#define MAX_RESPONSE_TIME 10000  // millis to wait for a response
#define MAX_MESSAGE_SIZE 1000
#define POWER_COMMAND_MSG 17
#define MESSAGE_CHECK_INTERVAL 15000UL  // 15 secs
#define UNDER_COMMAND_CHECK_INTERVAL 1000UL //1 sec interval afer a command is received 
#define COMMAND_POLL_TIMER 60 //Number of secs to poll control station more frequently for commands (triggered when a command is received)
#define NETWORK_CHECK_INTERVAL 5000UL  // Check network up every 5secs when down
#define BUFF_SIZE MAX_MESSAGE_SIZE + 6
uint8_t buff[BUFF_SIZE];
#define MAX_GET_MSG_SIZE 64  // Max size of dynamic get msg with params
char getMessageBuff[MAX_GET_MSG_SIZE];

#define RED_LED 1
#define GREEN_LED 3
#define SDA_PIN 2
#define SCL_PIN 0

#define DEBUG 0

const char* ssid     = SSID_NAME;
const char* password = PASSWORD;
const String GET_STR = "GET /powerc?s=20&r=";
const String HTTP_STR = " HTTP/0.9";
const int RELAY_ADDR = 0x11;

char CONTENT_LENGTH[] = "Content-Length: ";
char HTTP_09[] = "HTTP/0.9 ";

struct PowerControlMsg {
  int8_t relay;
  int8_t state;
};

enum Colours {
  OFF,
  RED,
  ORANGE,
  GREEN,
};

// Initialize the client library
WiFiClient client;
// Initialise Relay
Multi_Channel_Relay relay;

bool networkUp = false;
unsigned long commandPollTimer = 0;
Colours currentLEDColour;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // Initialize the LED_BUILTIN pin as an output
  digitalWrite(LED_BUILTIN, LOW);   // Turn the built in LED to show ESP01 has booted
  //Note: Initialising Wire() will turn off the Builtin LED as it is on GPIO2, which becomes the SDA pin 
  //So delay 500ms to let built in LED flash on briefly
  delay(500);
  if (!DEBUG) {
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    setLED(ORANGE);
  } else {
    Serial.begin(115200);
    Serial.println("Initialising Wire library for I2C");
  }
  Wire.begin(SDA_PIN, SCL_PIN);
  // Wire.begin();

  // Set Relay I2C address and start relay
  if (DEBUG) Serial.println("Initialising Relay library");
  relay.begin(RELAY_ADDR);
  /* Read firmware  version */
  if (DEBUG) {
    Serial.print("Relay firmware version: ");
    Serial.print("0x");
    Serial.print(relay.getFirmwareVersion(), HEX);
    Serial.println();
  }

  // listNetworks();
  if (DEBUG) Serial.println("Starting Wifi");
  WiFi.mode(WIFI_STA);
  WiFi.config(clientIP, dns, gateway);
  WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED) {
  //   if (DEBUG) Serial.printf("Connection status: %d\n", WiFi.status());
  //   delay(500);
  // }

  if (DEBUG) {
    printWifiStatus();
    Serial.print("IP Address: ");
    Serial.println(WiFi.localIP());
  }
  networkUp = true;
  if (DEBUG) Serial.println("Entering loop");
  if (!DEBUG) {
    setLED(GREEN);
    delay(1000); //Let green LED be recoginised
  }
}


void loop() {
  // Check for any messages
  if (checkNetworkUp()) {
    uint8_t relayStatus = relay.getChannelState();
    if (DEBUG) {
      Serial.print("Relay Status: ");
      Serial.println(relayStatus);
    }
    uint16_t respLen = getMessage(relayStatus);
    if (respLen > 0) {
      processMessage();
    }
  }
  if (commandPollTimer > 0) {
    delay(UNDER_COMMAND_CHECK_INTERVAL);
    commandPollTimer--;
  } else if (networkUp) {
    //Network up and no message - poll at msg check interval
    delay(MESSAGE_CHECK_INTERVAL);
  } else {
    //Network down - poll at network check interval
    delay(NETWORK_CHECK_INTERVAL);
  }
}

  void setLED(Colours colour) {
    currentLEDColour = colour;
    switch (colour) {
      case (OFF):
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, LOW);
        break;
      case (RED):
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, LOW);
        break;
      case (ORANGE):
        digitalWrite(RED_LED, HIGH);
        digitalWrite(GREEN_LED, HIGH);
        break;
      case (GREEN):
        digitalWrite(RED_LED, LOW);
        digitalWrite(GREEN_LED, HIGH);
        break;
    }
  }

void flickerLED() {
  Colours current = currentLEDColour;
  setLED(OFF);
  delay(50);
  setLED(ORANGE);
  delay(100);
  setLED(OFF);
  delay(50);
  setLED(current);
}

uint16_t getMessage(uint8_t relayStatus) {
  uint16_t respLen = 0;
  if (openTCP()) {
    drainWifi();
    if (!DEBUG) {
      flickerLED();
    } else {
      Serial.println("Sending HTTP Get");  //Zero first byte == success
    }
    sendHttpRequest(relayStatus);
    respLen = waitForHttpResponse();
    if (DEBUG) {
      if (respLen > 0) {
        Serial.print("Rx message: Length ");
        Serial.println(respLen);
        printBuff(respLen);
      } else {
        Serial.println("No relay command received");
      }
    }
  } else {
    // Server down error message
    if (!DEBUG) {
      setLED(ORANGE);
    } else {
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
  }
  return respLen;
}

void processMessage() {
  if (!DEBUG) {
    //Network and server up and got a valid response
    setLED(GREEN);
  } else {
    Serial.print("Rx Msg: ");
    Serial.println((int)buff[0]);
  }
  if (buff[0] == POWER_COMMAND_MSG) {
    commandPollTimer = COMMAND_POLL_TIMER;
    PowerControlMsg *pc = (PowerControlMsg *)&buff[4];  // Start of content
    if (DEBUG) {
      Serial.print("Relay: ");
      Serial.print(pc->relay);
      Serial.print(" State: ");
      Serial.println(pc->state);
    }
    if ((pc->state == 1 || pc->state == 0) && (pc->relay >= 1 && pc->relay <= 4)) {
      //Valid command
      if (pc->state == 1) {
        relay.turn_on_channel(pc->relay);
      } else {
        relay.turn_off_channel(pc->relay);
      }
    } else {
      //Invalid
      if (!DEBUG) {
        setLED(ORANGE);
      } else {
        Serial.print("Invalid command received: Relay no: ");
        Serial.print(pc->relay);
        Serial.print(" State: ");
        Serial.println(pc->state);
      }
    }
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
  byte mac[6];
  WiFi.macAddress(mac);
  Serial.print(" MAC: ");
  Serial.print(mac[0], HEX);
  Serial.print(":");
  Serial.print(mac[1], HEX);
  Serial.print(":");
  Serial.print(mac[2], HEX);
  Serial.print(":");
  Serial.print(mac[3], HEX);
  Serial.print(":");
  Serial.print(mac[4], HEX);
  Serial.print(":");
  Serial.print(mac[5], HEX);
}

bool checkNetworkUp() {
  // Check network up
  networkUp = (WiFi.status() == WL_CONNECTED);
  if (!networkUp) {
    // Wifi error
    if (!DEBUG) {
      setLED(RED);
    } else {
      Serial.print("1 Wifi Down: ");
      Serial.print(getWifiStatus());
      Serial.println();
    }
  }
  return networkUp;
}

bool openTCP() {
  bool connected = false;
  if (client.connect(serverAddr, PORT)) {
    connected = true;
  }
  return connected;
}

void sendHttpRequest(uint8_t relayStatus) {
  // Make a HTTP request
  //Wifi library wants a String
  String msg = GET_STR + relayStatus + HTTP_STR;
  client.println(msg);
  client.println();
  client.println();
  if (DEBUG) Serial.println(msg);
}

uint16_t waitForHttpResponse() {
  unsigned long start = millis();
  uint16_t bufPos = 0;
  uint16_t contentLen = 0;
  unsigned long waitTime = 0;
  bool gotRespCode = false;
  bool gotMsg = false;
  while (waitTime < MAX_RESPONSE_TIME && !gotMsg) {
    uint16_t len = receiveClientData(bufPos);
    if (len > 0) {
      bufPos += len;
      if (!gotRespCode) {
        uint16_t pos = findStringInBuff(buff, HTTP_09, sizeof(HTTP_09) - 1, bufPos);
        if (pos != -1 && (pos + 3) < bufPos) {
          pos += sizeof(HTTP_09) - 1;
          // Retrieve respCode
          char respCode[] = "   ";
          uint16_t endPos = pos;
          for (; endPos < (pos + 3); endPos++) {
            respCode[endPos - pos] = buff[endPos];
          }
          respCode[endPos - pos] = '\0';  // null terminate
          if (DEBUG) {
            Serial.print("Resp Code:");
            Serial.print((uint8_t)respCode[0], HEX);
            Serial.print((uint8_t)respCode[1], HEX);
            Serial.println((uint8_t)respCode[2], HEX);
          }
          uint16_t resp = atoi(&respCode[0]);
          if (!DEBUG) {
            setLED(ORANGE);
          } else {
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
        if (pos != -1 && (pos + 4) < bufPos) {
          pos += sizeof(CONTENT_LENGTH) - 1;
          // Retrieve content length
          char lenStr[] = "    ";
          uint16_t endPos = pos;
          for (; endPos < (pos + 3); endPos++) {
            if (int(buff[endPos]) == 13) {
              endPos++;
              break;  // Reached end of line
            }
            lenStr[endPos - pos] = buff[endPos];
          }
          lenStr[endPos - pos] = '\0';  // null terminate
          contentLen = atoi(&lenStr[0]);
          if (DEBUG) {
            Serial.print("Content Len Str:");
            Serial.print((uint8_t)lenStr[0], HEX);
            Serial.print((uint8_t)lenStr[1], HEX);
            Serial.print((uint8_t)lenStr[2], HEX);
            Serial.print("Content Len:");
            Serial.println(contentLen);
          }
          if (contentLen > 0 && bufPos > endPos + 3) {
            endPos += 3;  // 0xd, 0xa, 0xd, 0xa at end of content str and before content
            // shift bytes down in buff
            shiftBuffDown(buff, MAX_MESSAGE_SIZE, endPos, bufPos);  // Now we should just have the message content in the buff..
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
    if (!gotMsg) {
      // Wait for a bit for the next bytes to arrive
      delay(10);
    }
    waitTime = millis() - start;
  }
  if (waitTime >= MAX_RESPONSE_TIME && !gotMsg) {
    if (!DEBUG) {
      setLED(ORANGE);
    } else {
      Serial.print("1Msg Receive timeout:");
      Serial.print(" Rx bytes:");
      Serial.print(bufPos);
      Serial.print(" contentLen:");
      Serial.print(contentLen);
      Serial.println("EOL");
      printBuff(bufPos);
    }
  }
  return contentLen;
}

uint16_t findStringInBuff(uint8_t bf[], char str[], uint8_t strLen, uint16_t maxPos) {
  uint16_t startPos = -1;
  for (int i = 0; (i + strLen) < maxPos; i++) {
    if (bf[i] == str[0]) {
      startPos = i;
      uint16_t strPos = 0;
      uint16_t endStr = i + strLen;
      int j = i;
      for (; j < endStr && j < maxPos; j++) {
        if (bf[j] != str[strPos++]) {
          break;
        }
      }
      if (j >= endStr) {  //All chars matched
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
  }
  return len;
}

void shiftBuffDown(uint8_t bf[], uint16_t buffSize, uint16_t pos, uint16_t maxPos) {
  uint8_t p = pos;
  for (int i = 0; i < maxPos; i++) {
    if (p < buffSize) {
      bf[i] = bf[p++];
    } else {
      bf[i] = 0;
    }
  }
  for (int i = maxPos; i < buffSize; i++) {
    bf[i] = 0;
  }
}

void drainWifi() {
  delay(10);
  int total = 0;
  while (uint16_t len = receiveClientData(0) > 0) {
    total += len;
    delay(10);
  }
  resetBuffer();
}

void resetBuffer() {
  // Reset receive buffer
  for (uint32_t i = 0; i < MAX_MESSAGE_SIZE; i++) {
    buff[i] = 0;
  }
}

void printBuff(uint32_t pos) {
  Serial.print("Hex:");
  for (uint32_t i = 0; i < pos && i < MAX_MESSAGE_SIZE; i++) {
    Serial.print((uint8_t)buff[i], HEX);
    Serial.print(',');
  }
  Serial.print("\nText:");
  for (uint32_t i = 0; i < pos && i < MAX_MESSAGE_SIZE; i++) {
    Serial.print((char)buff[i]);
  }
  Serial.println();
}

void listNetworks() {
  // scan for nearby networks:
  Serial.println("** Scan Networks **");
  int numSsid = WiFi.scanNetworks();
  if (numSsid == -1) {
    Serial.println("Couldn't get a wifi connection");
    while (true)
      ;
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