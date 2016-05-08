/**
 * \file
 *       Arduino Chicken Coop Controller
 * \Original author
 *       Will Vincent <will@willvincent.com>
 * \Forked by
 *       Pablo Sanchez <pablo@pablo-sanchez.com>
 * \ Description
 * This Sketch is designed to work on a ESP-8266 NodeMCU V0.9 board
 */

#include <Wire.h>
#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <PubSubClient.h>         // https://github.com/knolleary/pubsubclient
#include "RTClib.h"

// print debug messages or not to serial
const int Debugging = 1;

// Digital & analog pins for various components

const int lightSense    = A0; // Light Sensor
const int doorOpen      = 12; // MotorA CCW UP
const int doorClose     = 14; // MotorA CW DOWN
const int doorTop       = 5; // Reed Switch
const int doorBottom    = 4; // Reed Switch
const int rtcSDA        = 0; // Real Time Clock SDA (i2c ESP8266 NodeMCU 0.9)
const int rtcSCL        = 2; // Real Time Clock SCL (i2c ESP8266 NodeMCU 0.9)

// WIFI Settings
#define wHost "secure2"
#define wPass "e28fiibgtybs"

// MQTT Host Settings
#define mHost "isore.net"
#define mPort 1883
#define mClientID "coop-duino"
//#define mUsername "chicken_coop"
//#define mPassword "secret"

// MQTT Subscription Channels
#define sTime    "time/beacon"
#define sRemote  "coop/remotetrigger"
#define sSunRise "sun/rise"
#define sSunSet  "sun/set"

// MQTT Publish Channels
#define pLight  "coop/brightness"
#define pStatus "coop/status"

// DOOR motion
#define GO     0
#define STOP   1

// Misc Settings
const unsigned long millisPerDay    = 86400000; // Milliseconds per day
const unsigned long debounceWait    =      200; // Door debounce timer (200 ms)
const unsigned long lightReadRate   =    10000; // How often to read light level (5 sec)
const unsigned long remoteOverride  =   600000; // Length of time to lockout readings. (10 min)
const unsigned long publishInterval =    60000; // How often to publish light sensor readings. (1 min)
const int           lsLow           =        0; // Light Sensor lowest reading
const int           lsHigh          =      900; // Light Sensor highest reading

// Night time lockout to prevent reaction to light sensor readings if an exterior light source causes
// a reading otherwise bright enough to activate the interior light and/or door.
const boolean       nightLock      =      true; // Enable night time lockout


/*************************************************
       DO   NOT   EDIT   BELOW   THIS   LINE
 *************************************************/

// Runtime variables
int           nightLockStart     =    22; // Hour (in 24hr time) to initiate night time lockout (10pm)
int           nightLockEnd       =     4; // Hour (in 24hr time) to end night time lockout (4am)
unsigned long lastPublish        =     0;
unsigned long lastDebounce       =     0;
unsigned long lastLightRead      =     0;
unsigned long lastRTCSync        =     0;
unsigned long remoteLockStart    =     0;
String        doorState          =    "Uninitialzed"; // Values will be one of: closed, closing, open, opening, unknown
String        doorStatePrev      =    "";
int           brightness         =     0;
int           doorTopVal         =     0;
int           doorTopVal2        =     0;
int           doorTopState       =     0;
int           doorTopPrev        =     0;
int           doorBottomVal      =     0;
int           doorBottomVal2     =     0;
int           doorBottomState    =     0;
int           doorBottomPrev     =     0;
uint32_t      bootTime           =     0;

RTC_DS1307 RTC;
WiFiClient espClient;
PubSubClient mqtt(espClient);

boolean wifiConnected = false;

/**
 * Manage WIFI connection
 */
void wifiCb(WiFiEvent_t event) {
  Serial.printf("\n[WiFi-event] event: %d\n", event);

  switch(event) {
      case WIFI_EVENT_STAMODE_GOT_IP:
         if (Debugging) {
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
         }
         mqtt.connect(mClientID);
         wifiConnected = true;
         break;
     case WIFI_EVENT_STAMODE_DISCONNECTED:
         if (Debugging) {
            Serial.println("WiFi lost connection");
         }
         wifiConnected = false;
         mqtt.disconnect();
         break;
    }
}

/**
 * MQTT Connection event handler.
 *
 * Subscribes to desired channels
 */
void mqttConnected() {
  if (wifiConnected) {
     if (Debugging) {
       Serial.println("MQTT Connected");
     }
     // Subscribe to time beacon channel to keep RTC up to date.
     mqtt.subscribe(sTime, 0);

     // Subscribe to remote trigger channel to allow remote control of chicken coop
     mqtt.subscribe(sRemote, 1);

     // Subscribe to sunrise/set updates
     mqtt.subscribe(sSunRise, 1);
     mqtt.subscribe(sSunSet, 1);
  
     // Publish that we're online!
     mqtt.publish("client/online", "1");
  } else {
     if (Debugging) {
       Serial.println("MQTT NOT Connected because WIFI is not connected");
     }
  }
}

/**
 * MQTT Disconnect event handler.
 */
 /*
void mqttDisconnected(void* response) {
  if (Debugging) {
    Serial.println("MQTT Disconnected");
  }
}
*/

/**
 * MQTT Publish event handler.
 *
 * We don't really need this, but it allows logging of any sent
 * messages for debugging purposes.
 */
/*
 void mqttPublished(char* topic, byte* data, unsigned int length) {
  //RESPONSE res(response);
  //String topic = res.popString();
  //String data  = res.popString();
  if (Debugging) {
    if (topic != "time/beacon") {
      Serial.print("MQTT Data published to channel '");
      Serial.print(topic);
      Serial.print("': ");
      Serial.println(data);
    }
  }
}
*/ 

/**
 * Handle incoming MQTT messages.
 *
 * This allows us to remotely trigger events via WIFI!
 */
void mqttData(char* topic, byte* payload, unsigned int plen) {
  
  byte* p = (byte *)malloc(plen+1);
  memset(p,'\0',plen+1);
  memcpy(p,payload,plen);
  String data  = String((char *) p);
  free(p);
  
  if(Debugging) {
     Serial.print("mqttTopic*Len*Data: |");
     Serial.print(topic);
     Serial.print("| * |");
     Serial.print(plen);
     Serial.print("| * |");
     Serial.print(data);
     Serial.println("|");
  }

  if (strcmp(topic,sRemote)==0) {
    // If door movement is triggered, toggle door state to
    // opening or closing based on current state.
    // If door is currently moving, the trigger is ignored.
    if (data == "unlock") {  // this will immediately put the system back into normal operation (i.e., using the light sensor)
      remoteLockStart = 0;
      if(Debugging) {
        Serial.println("remoteLockStart unset");
      }      
    }
    if (data == "door") {
      if(Debugging) {
        Serial.println("remoteLockStart set");
      }
      if (doorState == "open") {
        doorState = "closing";
        remoteLockStart = millis();
      }
      else if (doorState == "closed") {
        doorState = "opening";
        remoteLockStart = millis();
      }
    }
  }


  // Adjust sunrise/set times for nightlock
  if (strcmp(topic,sSunRise)==0) {
    nightLockEnd = atoi(data.c_str());
    if (Debugging) {
      Serial.print("Night lock end updated to: ");
      Serial.println(nightLockEnd);
    }
  }
  
  if (strcmp(topic,sSunSet)==0) {
    nightLockStart = atoi(data.c_str());
    if (Debugging) {
      Serial.print("Night lock start updated to: ");
      Serial.println(nightLockStart);
    }
  }

  // Sync RTC to time beacon once/day
  if (strcmp(topic,sTime)==0) {
    if (lastRTCSync == 0 || ((unsigned long)(millis() - lastRTCSync) > millisPerDay)) {
      RTC.adjust(strtoul(data.c_str(), NULL, 0));
      lastRTCSync = millis();
      if (Debugging) {
        DateTime now = RTC.now();
        char dateStr[11];
        char timeStr[9];

        sprintf(dateStr, "%02d/%02d/%04d", now.month(), now.day(), now.year());
        int hr = now.hour();
        boolean ampm = false;
        if (hr > 12) {
          hr = hr - 12;
          ampm = true;
        }
        else if (hr == 12) {
          ampm = true;
        }
        else if (hr == 0) {
          hr = 12;
        }
        sprintf(timeStr, "%02d:%02d:%02d", hr, now.minute(), now.second());
        Serial.println("RTC Updated:");
        Serial.println(dateStr);
        Serial.print(timeStr);
        if (ampm) {
          Serial.println("pm");
        }
        else {
          Serial.println("am");
        }

      }
    }
  }
}

/**
 * Publish Temp and Light Readings.
 */
void publishReadings() {

  // Since the webapp likes to get confused, lets remind them we're
  // online every time we update brightness and temp readings...

  if (wifiConnected) {
     mqtt.publish("client/online", "1");
  }
  if (Debugging) {
      Serial.print("mqtt publish. Status: ");
      Serial.println(mqtt.state());
  }
}

/**
 * Handle movement of the door
 */
void doorMove() {
  debounceDoor();
  doorStatePrev = doorState;
  if (doorState == "closed" || doorState == "closing") {
    if (doorBottomState != 0) {
      // Door isn't closed, run motor until it is.
      digitalWrite(doorClose, GO);
      digitalWrite(doorOpen, STOP);
    }
    else {
      // Door is closed, stop motor
      digitalWrite(doorClose, STOP);
      digitalWrite(doorOpen, STOP);
      doorState = "closed";
      if (doorStatePrev != doorState && wifiConnected) {
        mqtt.publish("coop/status", "door|closed");
      }
    }
  }
  else {
    if (doorTopState != 0) {
      // Door isn't open, run motor until it is.
      digitalWrite(doorClose, STOP);
      digitalWrite(doorOpen, GO);
    }
    else {
      // Door is open, stop motor.
      digitalWrite(doorClose, STOP);
      digitalWrite(doorOpen, STOP);
      doorState = "open";
      if (doorStatePrev != doorState && wifiConnected) {
        mqtt.publish("coop/status", "door|open");
      }
    }
  }
}

/**
 * Read current sensor data
 */
void readSensors() {
  if (lastLightRead == 0 || (unsigned long)millis() - lastLightRead > lightReadRate) {
    // Read light sensor and convert to brightness percentage
    brightness = analogRead(lightSense);
    brightness = map(brightness, 0, 1023, 0, 100);  // Remap value to a 0-100 scale
    brightness = constrain(brightness, 0, 100);     // constrain value to 0-100 scale
    lastLightRead = millis();
    
    char buf[100];
    String pubString = String(brightness);; 
    pubString.toCharArray(buf, pubString.length()+1);
    if (wifiConnected) { 
       mqtt.publish(pLight, buf);
    }

    if (Debugging) {
      Serial.print("Brightness is ");
      Serial.println(brightness);
    }

  }
}

/**
 * Respond to updated sensor data.
 */
void handleSensorReadings() {
  
  // Light based reactions
  // ---------------------

  // Fetch current time from RTC

  DateTime now = RTC.now();
  // If nightlock is enabled, and we are within the designated time period, simply
  // ensure door is closed.
  //if(Debugging) {
  //  Serial.printf("Nightlock: %i, Time: %0d:%0d\n",nightLock, now.hour(),now.minute());
  //}
  if (nightLock && ( (now.hour() >= nightLockStart || now.hour() <= nightLockEnd)) ) {

    // Close door if it is open
    if (doorState == "open") {
      if (Debugging) {
        Serial.println("NIGHTLOCK ENABLED: Closing door.");
      }
      doorState = "closing";
      if (wifiConnected) {
        mqtt.publish(pStatus, "door|closing");
      }
    }
  }
  // Otherwise, handle brightness level based reactions

  // NOTE: We need a bit of a gap between these thresholds to prevent
  // bouncing if light readings fluctuate by a percentage or two.
  else {
    // Open door when brightness level is greater than 5%
    if (brightness >= 5) {
      if (doorState == "closed") {
        if (Debugging) {
          Serial.println("Opening door.");
        }
        doorState = "opening";
        if (wifiConnected) {
          mqtt.publish(pStatus, "door|opening");
        }
      }
    }
    // Otherwise, close door when light level falls below 2%.
    else if (brightness < 2) {
      if (doorState == "open") {
        if (Debugging) {
          Serial.println("Closing door.");
        }
        doorState = "closing";
        if (wifiConnected) {
          mqtt.publish(pStatus, "door|closing");
        }
      }
    }
  }
}

/**
 * Door switch debouncer
 */
void debounceDoor() {
  doorTopVal     = digitalRead(doorTop);
  doorBottomVal  = digitalRead(doorBottom);
  doorTopPrev    = doorTopState;
  doorBottomPrev = doorBottomState;

  if ((unsigned long)(millis() - lastDebounce) > debounceWait) {
    doorTopVal2    = digitalRead(doorTop);
    doorBottomVal2 = digitalRead(doorBottom);
    if (doorTopVal == doorTopVal2) {
      if (doorTopVal != doorTopState) {
        doorTopState = doorTopVal;
        if (doorTopState == 0) {
          doorState = "open";
          if (doorTopPrev != doorTopState) {
            if (Debugging) {
              Serial.println("Door open.");
              Serial.printf("wifiConnected: %i\n",wifiConnected);
            }
            if (wifiConnected) {
              mqtt.publish(pStatus, "door|open");
            }
          }
        }
      }
    }
    if (doorBottomVal == doorBottomVal2) {
      if (doorBottomVal != doorBottomState) {
        doorBottomState = doorBottomVal;
        if (doorBottomState == 0) {
          doorState = "closed";
          if (doorBottomPrev != doorBottomState) {
            if (Debugging) {
              Serial.println("Door closed.");
            }
            if (wifiConnected) {
              mqtt.publish(pStatus, "door|closed");
            }
          }
        }
      }
    }
    lastDebounce = millis();
  }
}

/**
 * Initialization on startup.
 */
void setup() {
  if (Debugging) {
    Serial.begin(115200);
    Serial.println("Initialising...");
    Serial.println("Setting motor status to STOP");
  }

  // Set the outputs
  digitalWrite(doorOpen, STOP);
  digitalWrite(doorClose, STOP);
  digitalWrite(doorTop, HIGH);
  digitalWrite(doorBottom, HIGH);
  
  pinMode(doorOpen, OUTPUT);
  pinMode(doorClose, OUTPUT);
  pinMode(doorTop, INPUT_PULLUP);
  pinMode(doorBottom, INPUT_PULLUP);

// Again, just in case
  digitalWrite(doorOpen, STOP);
  digitalWrite(doorClose, STOP);
  digitalWrite(doorTop, HIGH);
  digitalWrite(doorBottom, HIGH);

  
  Wire.begin(rtcSDA,rtcSCL);
  if (! RTC.begin()) {
    Serial.println("Couldn't find RTC");
    while (1);
  }
  if (! RTC.isrunning() ) {
    //clockmode = NOT_SET;
    if (Debugging) {
       Serial.println("RTC is not running");
    }
    RTC.adjust(DateTime(__DATE__, __TIME__));  // try kick starting the clock with the compile time
  }
  if (Debugging) {
    DateTime now = RTC.now();
    Serial.printf("RTC time is: %0d:%0d\n",now.hour(),now.minute());    
  }
  
  if (Debugging) {
    Serial.println("ARDUINO: Setup WIFI");
  }
  WiFi.disconnect();
  
  delay(1000);
  
  WiFi.onEvent(wifiCb);
  WiFi.begin(wHost, wPass);
  
  while (WiFi.status() != WL_CONNECTED) { // &&  wifitry >0 ) {
    delay(500);
    if (Debugging) {
       Serial.print(".");
    }
  }
  if (WiFi.status() == WL_CONNECTED) {
    if (Debugging) {
       Serial.println("");
       Serial.println("WiFi connected");  
       Serial.println("IP address: ");
       Serial.println(WiFi.localIP());
    }
    wifiConnected = true;
  } else {
    if (Debugging) {
       Serial.println("");
       Serial.println("WiFi UNconnected");
       wifiConnected = false;
    }
  }
  
  if (Debugging) {
    Serial.println("ARDUINO: Setup MQTT client");
    Serial.print("Client|port: ");
    Serial.print(mHost);
    Serial.print("|");
    Serial.println(mPort);
  }
  
  mqtt.setCallback(&mqttData);
  mqtt.setServer(mHost, mPort);  // client_id, port
  mqtt.connect(mClientID);

  delay(500);
  
  if ( !mqtt.connected() ) { 
    if (Debugging) {
      Serial.println("ARDUINO: Failed to setup MQTT");
    }   
  }

  if (Debugging) {
    Serial.print("MQTT Status: ");
    Serial.println(mqtt.state());    
  }

  mqttConnected();

  //mqtt.connectedCb.attach(&mqttConnected);
  //mqtt.disconnectedCb.attach(&mqttDisconnected);
  //mqtt.publishedCb.attach(&mqttPublished);
  //mqtt.setCallback(&mqttData);

  // Findout door status
  int dt=digitalRead(doorTop);
  int db=digitalRead(doorBottom);
  switch (dt + db) {
    case 0:
       doorState = "broken";
       if (Debugging) {
           Serial.println("Door broken. Opened and closed simultaneously. Halting");
       }
       if (wifiConnected) {
          mqtt.publish(pStatus,"Door sensors are insane. Halting.");
       }
       while(1);
       break;
    case 1:
       if(dt==0){
          doorState="open";
          if (wifiConnected) {
             mqtt.publish("coop/status", "door|open");
          }
       } else {
          doorState="closed";
          if (wifiConnected) {
             mqtt.publish("coop/status", "door|closed");
          }
       }
       break;
    case 2:
       doorState = "unknown";
       if (wifiConnected) {
          mqtt.publish("coop/status", "door|unknown");
       }
       break;
  }

  if (Debugging) {
    Serial.print("Initial door state: ");
    Serial.println(doorState);
  }
}

/**
 * Main program loop
 */
void loop() {
  mqtt.loop();
  // 5 second pause on initial startup to let devices settle, wifi connect, etc.
  if (millis() > 5000) {
//    Serial.println("Loop...");
 
    // Read new data from sensors
    readSensors();
    
    if (remoteLockStart == 0 ||
        (unsigned long)(millis() - remoteLockStart) > remoteOverride) {
      // Respond ot sensor data
      handleSensorReadings();
    }

    // Only publish new sensor data if it's been long enough since the last reading.
    if (lastPublish == 0 || (unsigned long)(millis() - lastPublish) > publishInterval) {
      if (wifiConnected) {
        publishReadings();
      }
      lastPublish = millis();
    }

    // Move the door as needed
    doorMove();
  }
}
