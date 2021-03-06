/**
 * \file
 *       Arduino Chicken Coop Controller
 * \Original author
 *       Will Vincent <will@willvincent.com>
 * \Forked by
 *       Pablo Sanchez <pablo@pablo-sanchez.com>
 * \ Description
 * This Sketch is designed to work on a ESP-8266 MUANODE 0.9 board
 */

#include <Wire.h>                 // https://www.arduino.cc/en/Reference/Wire
#include <ESP8266WiFi.h>          // https://github.com/esp8266/Arduino
#include <PubSubClient.h>         // https://github.com/knolleary/pubsubclient
#include <Bounce2.h>              // https://github.com/thomasfredericks/Bounce2
#include <RTClib.h>               // https://github.com/adafruit/RTClib
#include <LiquidCrystal_I2C.h>
#include "secrets.h"              // holds wifi credentials & MQTT credentials
#include "pins.h"                 // holds pin definitions

// print debug messages or not to serial
int Debugging = 1;

//// Digital & analog pins for various components
//
//const int lightSense    = A0; // Light Sensor
//
//const int doorTop       = D1; // Reed Switch
//const int doorBottom    = D2; // Reed Switch
//const int rtcSDA        = D3; // Real Time Clock SDA (i2c ESP8266 NodeMCU 0.9)
//const int rtcSCL        = D4; // Real Time Clock SCL (i2c ESP8266 NodeMCU 0.9)
//const int doorClose     = D5; // MotorA CW DOWN
//const int doorOpen      = D6; // MotorA CCW UP

// WIFI Settings ** Moved to secrets.h **
//#define wHost "wifi"
//#define wPass "secret"

// MQTT Host Settings
//#define mHost "mqtt_host"
//#define mPort 1883
//#define mClientID "coop-duino"
//#define mUsername "chicken_coop"
//#define mPassword "secret"

// MQTT Subscription Channels
#define sTime    "time/beacon"
#define sRemote  "coop2/remotetrigger"
#define sSunRise "sun/rise"
#define sSunSet  "sun/set"

// MQTT Publish Channels
#define pLight  "coop2/brightness"
#define pStatus "coop2/status"

// DOOR motion
#define GO     0
#define STOP   1

// Misc Settings
const unsigned long millisPerDay    = 86400000; // Milliseconds per day
const unsigned long lightReadRate   =    30000; // How often to read light level (30 sec)
const unsigned long remoteOverride  =   600000; // Length of time to lockout readings. (10 min)
const unsigned long publishInterval =   600000; // How often to publish light sensor readings. (10 min)
const unsigned long maxMotorOn      =    15000; // Maximum time for the motor to be on (15 seconds)

// Night time lockout to prevent reaction to light sensor readings if an exterior light source causes
// a reading otherwise bright enough to activate the interior light and/or door.
const boolean       nightLock      =      true; // Enable night time lockout

/*************************************************
       DO   NOT   EDIT   BELOW   THIS   LINE
 *************************************************/

// Runtime variables
int           nightLockStart     =    22; // Hour (in 24hr time) to initiate night time lockout (10pm)
int           nightLockEnd       =     4; // Hour (in 24hr time) to end night time lockout (4am)
unsigned long lastDebounce       =     0;
unsigned long lastLightRead      =     0;
unsigned long lastRTCSync        =     0;
unsigned long remoteLockStart    =     0;
unsigned long motorRunning       =     0;  // how long has the motor been on
String        doorState          =    "Uninitialzed"; // Values will be one of: closed, closing, open, opening, unknown
String        doorStatePrev      =    "";
int           brightness         =     0;
int           openBright         =    40; // brightness to wait until opening the door
int           closeBright        =    35; // brightness to wait until closing the door
int           doorTopVal         =     0;
int           doorTopVal2        =     0;
int           doorTopState       =     0;
int           doorTopPrev        =     0;
int           doorBottomVal      =     0;
int           doorBottomVal2     =     0;
int           doorBottomState    =     0;
int           doorBottomPrev     =     0;
uint32_t      bootTime           =     0;
int           motorHalted        =     0; // Motor protection in case of run away condition
char          mqtt_msg_buf[100];
DateTime      now;

// Define the hardware components
RTC_DS1307 RTC;
EspClass esp;
WiFiClient espClient;
LiquidCrystal_I2C lcd(0x27, 16, 2);
PubSubClient mqtt(espClient);

// Setup debounce software
Bounce debounceTop = Bounce();
Bounce debounceBot = Bounce();

boolean wifiConnected = false;

/**
 * Manage WIFI connection
 */
void wifiCb(WiFiEvent_t event) {
  if (Debugging) {
    Serial.printf("\n[WiFi-event] event: %d\n", event);
  }

  switch(event) {
      case WIFI_EVENT_STAMODE_GOT_IP:
         if (Debugging) {
            Serial.println("WiFi connected");
            Serial.println("IP address: ");
            Serial.println(WiFi.localIP());
         }
         mqtt.connect(mClientID);
         if ( !mqtt.connected() ) {
            delay(100);
            mqtt.connect(mClientID);
         }
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
     mqtt.publish(pStatus, "mqttOnline");
  } else {
     if (Debugging) {
       Serial.println("MQTT NOT Connected because WIFI is not connected");
     }
  }
}

/**
 * Handle incoming MQTT messages.
 *
 * This allows us to remotely trigger events via WIFI!
 */
void mqttData(char* topic, byte* payload, unsigned int plen) {

  // Copy the payload to the MQTT message buffer
  memset(mqtt_msg_buf,'\0',plen+1);
  memcpy(mqtt_msg_buf,payload,plen);
  String data  = String((char *) mqtt_msg_buf);

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
    if (data == "debug") {
      Debugging = 1 - Debugging;
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
        now = RTC.now();
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
 * Handle movement of the door
 */
void doorMove() {

  if (motorHalted == 0 ) {
    if(motorRunning > 0) {

    if (motorRunning + maxMotorOn < millis()){
      digitalWrite(doorClose, STOP);
      digitalWrite(doorOpen, STOP);

      if (Debugging) {
        Serial.println("Motor on too long - Halting!");
      }

      if (doorStatePrev != doorState && wifiConnected) {
        mqtt.publish(pStatus, "door|runaway");
      }
      motorHalted = 1;
     }
  }

  doorStatePrev = doorState;
  if (doorState == "closed" || doorState == "closing") {
    if (doorBottomState != 0) {
      // Door isn't closed, run motor until it is.
      if(motorRunning == 0) {  // start your engines
        digitalWrite(doorClose, GO);
        digitalWrite(doorOpen, STOP);
        motorRunning = millis();
        if (Debugging) {
          Serial.printf("Motor CLOSE ON: %i\n", motorRunning);
        }
      }
    }
    else {
      if (motorRunning > 0) {
      // Door is closed, stop motor
      digitalWrite(doorClose, STOP);
      digitalWrite(doorOpen, STOP);
      doorState = "closed";
      motorRunning = 0; // stop the motor running counter
      if (doorStatePrev != doorState && wifiConnected) {
        mqtt.publish(pStatus, "door|closed");
      }
      if (Debugging) {
          Serial.println("Motor CLOSE OFF");
      }
      }
    }
  }
  else {
    if (doorTopState != 0) {
      if(motorRunning == 0) {  // start your engines
        // Door isn't open, run motor until it is.
        digitalWrite(doorClose, STOP);
        digitalWrite(doorOpen, GO);        motorRunning = millis();
        if (Debugging) {
          Serial.printf("Motor OPEN ON: %i \n",motorRunning);
        }
      }
    }
    else {
      // Door is open, stop motor.
      if (motorRunning > 0) {
      digitalWrite(doorClose, STOP);
      digitalWrite(doorOpen, STOP);
      doorState = "open";
      motorRunning = 0; // stop the motor running counter
      if (doorStatePrev != doorState && wifiConnected) {
        mqtt.publish(pStatus, "door|open");
      }
      if (Debugging) {
        Serial.println("Motor OPEN OFF");
      }
      }
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
        Serial.print("Free mem: ");
        Serial.println(esp.getFreeHeap());
        Serial.print("Millis: ");
        Serial.println(millis());
        Serial.print("MQTT State: ");
        Serial.println(mqtt.state());
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

  now = RTC.now();
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
    if (brightness >= openBright) {
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
    else if (brightness < closeBright) {
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
 * Initialization on startup.
 */
void setup() {
  if (Debugging) {
    Serial.begin(115200);
    Serial.println("Initialising...");
    Serial.println("Setting motor status to STOP");
    Serial.println(pStatus);
  }

  digitalWrite(doorOpen, STOP);  // Set the moto control relays
  digitalWrite(doorClose, STOP);
  pinMode(doorOpen, OUTPUT);
  pinMode(doorClose, OUTPUT);

  // Define the top & bottom sensors
  pinMode(doorTop, INPUT_PULLUP);
  debounceTop.attach(doorTop);
  debounceTop.interval(5);  //inteval in ms
  pinMode(doorBottom, INPUT_PULLUP);
  debounceBot.attach(doorBottom);
  debounceBot.interval(5);  //inteval in ms

  // Again, just in case
  digitalWrite(doorOpen, STOP);
  digitalWrite(doorClose, STOP);

  // Get the RTC going
  Wire.begin(rtcSDA,rtcSCL);
  lcd.init();
  lcd.init();
  lcd.backlight();
  lcd.print(" coopController ");
  lcd.setCursor(0,1);
  lcd.print(">Pablo  Sanchez<");
  // Set the outputs
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
    now = RTC.now();
    Serial.printf("RTC time is: %0d:%0d\n",now.hour(),now.minute());
  }

  if (Debugging) {
    Serial.println("ARDUINO: Setup WIFI");
  }

  // Wifi Connect
  WiFi.disconnect();

  delay(200);

  WiFi.begin(wHost, wPass);

  int wifitry = 60; // try for 30 seconds then move on
  while (WiFi.status() != WL_CONNECTED &&  wifitry >0 ) {
    delay(100);
    wifitry--;
    if (Debugging) {
       Serial.print(".");
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    wifiConnected = true;
    lcd.clear();
    lcd.print("IP:");
    lcd.print(WiFi.localIP());
    if (Debugging) {
       Serial.println("");
       Serial.println("WiFi connected");
       Serial.println("IP address: ");
       Serial.println(WiFi.localIP());
    }
  } else {
    wifiConnected = false;
    lcd.clear();
    lcd.print("No WiFi");
    if (Debugging) {
       Serial.println("");
       Serial.println("WiFi UNconnected");
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

  delay(100);
  if ( !mqtt.connected() ) {
     mqtt.connect(mClientID);
     delay(100);
  }

  if ( !mqtt.connected() ) {
    if (Debugging) {
      Serial.println("ARDUINO: Failed to setup MQTT");
    }
  }

  if (Debugging) {
    Serial.print("MQTT Status: ");
    Serial.println(mqtt.state());
  }

  WiFi.onEvent(wifiCb);

  mqttConnected();

  // Findout door status
  debounceTop.update();
  debounceBot.update();

  doorTopState    = debounceTop.read();
  doorBottomState = debounceBot.read();

  switch (doorTopState + doorBottomState) {
    case 0:
       doorState = "broken";
       if (Debugging) {
           Serial.println("Door broken. Opened and closed simultaneously. Halting");
       }
       if (wifiConnected) {
          lcd.clear();
          lcd.print("Motor Insane");
          lcd.setCursor(0,1);
          lcd.print("Halting.");
          mqtt.publish(pStatus,"door|insane");
       }
       while(1);
       break;
    case 1:
       if(doorTopState==0){
          doorState="open";
          if (wifiConnected) {
             mqtt.publish(pStatus, "door|open");
          }
       } else {
          doorState="closed";
          if (wifiConnected) {
             mqtt.publish(pStatus, "door|closed");
          }
       }
       break;
    case 2:
       doorState = "unknown";
       if (wifiConnected) {
          mqtt.publish(pStatus, "door|unknown");
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
  lcd.setCursor(0,1);
  now = RTC.now();
  char buf[16];
  sprintf(buf,"%02d:%02d",now.hour(),now.minute());
  lcd.print(buf);
  mqtt.loop();
  // 5 second pause on initial startup to let devices settle, wifi connect, etc.
  if (millis() > 5000) {

    // Update top & bottom switches
    debounceTop.update();
    doorTopState = debounceTop.read();
    debounceBot.update();
    doorBottomState = debounceBot.read();

    // Read new data from sensors
    readSensors();

    if (remoteLockStart == 0 ||
        (unsigned long)(millis() - remoteLockStart) > remoteOverride) {
      // Respond ot sensor data
      handleSensorReadings();
    }

    // Move the door as needed
    doorMove();
  }
}

