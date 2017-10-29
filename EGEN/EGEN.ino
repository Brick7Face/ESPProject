/**************************************************************
 *  This program contains the microcontroller program and the GUI
 *  in one. The GUI is setup through the header file "index.h"
 *  in HTML5 and CSS3 using a web socket. This enables data to
 *  flow both directions. The microcontroller hosts the website
 *  and a client can connect and send data to the microcontroller.
 *  The microcontroller send data as well, such as the IMU
 *  information, allowing the client to access onboard statistics.
 *  
 *  Author: Nate Tranel
 *  EGEN310R Cat's Conundrum Project
 * 
 *************************************************************/


/**************************************
 * Include the necessary libraries
 *************************************/
#include <ESP8266WiFi.h>
#include <ESP8266WiFiMulti.h>
#include <WebSocketsServer.h>
#include <Hash.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <WiFiClient.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <Servo.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include "index.h"

/**********************************************
 * Create instances of the necessary obects
 **********************************************/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();                                 //create motor shield object with default I2C address, motor object, and 
Adafruit_DCMotor *myMotor = AFMS.getMotor(3);                                       //stepper motor for servo. Number indicates the port
//Adafruit_StepperMotor *myMotorS = AFMS.getStepper(200, 1);                          //second number indicates port
Servo servo;

//Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);                                      // Use I2C, ID #1000 for the IMU sensor

/**********************************************
 * Define specs for internet connection
 **********************************************/
//static const char* ssid = "TestNet";
//static const char* password = "password123";
static const char ssid[] = "BillWiTheScienceFi-2G";
static const char password[] = "genderspectrum810";
MDNSResponder mdns;
ESP8266WiFiMulti WiFiMulti;

//host web server on port 80 (standard), web socket on port 81
ESP8266WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);

/**********************************************
 * Define constants to be used
 **********************************************/
const char LED[] = "LED";
const char FRONT[] = "FRONT";
const char BACK[] = "BACK";
const char RIGHT[] = "RIGHT";
const char LEFT[] = "LEFT";
const char STOP[] = "STOP";
const char SLOW[] = "SLOW";
int i = 20;
int j = 75;

/**********************************************
 * Define functions - no prototypes since the
 * are implemented here
 **********************************************/
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length)                                             //handles functions for when a client triggers a web socket event
{
  Serial.printf("webSocketEvent(%d, %d, ...)\r\n", num, type);
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("[%u] Disconnected!\r\n", num);
      break;
    case WStype_CONNECTED:
      {
        IPAddress ip = webSocket.remoteIP(num);
        Serial.printf("[%u] Connected from %d.%d.%d.%d url: %s\r\n", num, ip[0], ip[1], ip[2], ip[3], payload);
      }
      break;
    case WStype_TEXT:
      Serial.printf("[%u] get Text: %s\r\n", num, payload);
      if (strcmp(LED, (const char *)payload) == 0) {                            //case for if LED button was pressed
        handleLED();
      }
      if (strcmp(FRONT, (const char *)payload) == 0) {                          //case for if forward button was pressed
        int f = i + 30;
        int g = i + 10;
        if (f < 255) {
          i = i + 30;
        }
        else if (g < 255) {
          i = i + 10;
        }
        forward(i);
      }
      if (strcmp(BACK, (const char *)payload) == 0) {                           //case for if backward button was pressed
        int f = i + 20;
        int g = i + 10;
        if (f < 255) {
           i = i + 20;
        }
        else if (g < 255) {
          i = i + 10;
        }
        backward(i);
      }
      if (strcmp(RIGHT, (const char *)payload) == 0) {                           //case for if right button was pressed
        if (j > 82) {
          j = 82;
        }
        else j = 59;
        turn(j);
      }
      if (strcmp(LEFT, (const char *)payload) == 0) {                           //case for if left button was pressed
        if (j < 82) {
          j = 82;
        }
        else j = 105;
        turn(j);
      }
      if (strcmp(STOP, (const char *)payload) == 0) {                           //case for if left button was pressed
        quit();
      }
      if (strcmp(SLOW, (const char *)payload) == 0) {                           //case for if left button was pressed
        int f = i - 20;
        if (f > 0) {
          i = i - 20;
          myMotor->setSpeed(i);
        }
      }



      
      // send data to all connected clients
      webSocket.broadcastTXT(payload, length);
      break;
    case WStype_BIN:
    {
      Serial.printf("[%u] get binary length: %u\r\n", num, length);
      hexdump(payload, length);

      // echo data back to browser
      webSocket.sendBIN(num, payload, length);
      break;
    }
    default:
    {
      Serial.printf("Invalid WStype [%d]\r\n", type);
      break;
    }
  }
}


void handleRoot() {                                                                    //function for handling startup: root for root directory of website
  server.send(200, "text/html", MAIN_page);                                            //upon startup, load HTML page to server
}

void handleLED() {                                                                     //function to turn LED on and off
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));                                 //sets the state of the LED to the inverse of what it currently was)
}

void forward(uint8_t d) {                                                              //function for moving car forward
  myMotor->run(FORWARD);
  myMotor->setSpeed(d);
  digitalWrite(LED_BUILTIN, HIGH);                         
}

void backward(uint8_t d) {                                                             //function for moving car backward
  myMotor->run(BACKWARD);
  myMotor->setSpeed(d);
  digitalWrite(LED_BUILTIN, LOW);
}

void turn(int k) {
  servo.write(k);
}

void quit() {
  i = 0;
  myMotor->setSpeed(0);
}

/*
void configureSensor(void)                                                             //function to setup the IMU sensor
{
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);                          // 1.) Set the accelerometer range
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_6G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);                           // 2.) Set the magnetometer sensitivity
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);                        // 3.) Setup the gyroscope
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}



void displaySensorDetails(void)
{
  sensor_t accel, mag, gyro, temp;
  
  lsm.getSensor(&accel, &mag, &gyro, &temp);
  
  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(accel.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(accel.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(accel.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(accel.max_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Min Value:    ")); Serial.print(accel.min_value); Serial.println(F(" m/s^2"));
  Serial.print  (F("Resolution:   ")); Serial.print(accel.resolution); Serial.println(F(" m/s^2"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(mag.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(mag.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(mag.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(mag.max_value); Serial.println(F(" uT"));
  Serial.print  (F("Min Value:    ")); Serial.print(mag.min_value); Serial.println(F(" uT"));
  Serial.print  (F("Resolution:   ")); Serial.print(mag.resolution); Serial.println(F(" uT"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(gyro.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(gyro.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(gyro.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(gyro.max_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Min Value:    ")); Serial.print(gyro.min_value); Serial.println(F(" rad/s"));
  Serial.print  (F("Resolution:   ")); Serial.print(gyro.resolution); Serial.println(F(" rad/s"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));

  Serial.println(F("------------------------------------"));
  Serial.print  (F("Sensor:       ")); Serial.println(temp.name);
  Serial.print  (F("Driver Ver:   ")); Serial.println(temp.version);
  Serial.print  (F("Unique ID:    ")); Serial.println(temp.sensor_id);
  Serial.print  (F("Max Value:    ")); Serial.print(temp.max_value); Serial.println(F(" C"));
  Serial.print  (F("Min Value:    ")); Serial.print(temp.min_value); Serial.println(F(" C"));
  Serial.print  (F("Resolution:   ")); Serial.print(temp.resolution); Serial.println(F(" C"));  
  Serial.println(F("------------------------------------"));
  Serial.println(F(""));
  
  delay(500);
}


/**********************************************
 * Function to setup the program before loop
 **********************************************/
void setup() {
  Serial.begin(115200);                   //begin serial transmissions on 115200 baud
  Serial.println("");    
  pinMode(LED_BUILTIN, OUTPUT);           //initialize LED, set it to off
  digitalWrite(LED_BUILTIN, HIGH); 
  AFMS.begin();                           //create motorshield with the default frequency 1.6KHz
  uint8_t i = 0;                          //variable to track current speed


  myMotor->setSpeed(150);                 //Set the speed to start, from 0 (off) to 255 (max speed)
 // myMotorS->setSpeed(10);                 //Set servo RPM to 10
  myMotor->run(FORWARD);                  //turn on motor
  myMotor->run(RELEASE);
  servo.attach(2);
  servo.write(82);

  /*
  if(!lsm.begin())
  {
    /* There was a problem detecting the LSM9DS0 ... check your connections 
    Serial.print(F("Ooops, no LSM9DS0 detected ... Check your wiring or I2C ADDR!"));
    while(1);
  }
  Serial.println(F("Found LSM9DS0 9DOF"));
  /* Display some basic information on this sensor 
  displaySensorDetails();
  /* Setup the sensor gain and integration time 
  configureSensor();
  Serial.println("");
  */

  WiFiMulti.addAP(ssid, password);
            
  //WiFi.mode(WIFI_AP);                     //define wifi as access point
  //WiFi.softAP(ssid, password);            //initialize web server

  //IPAddress myIP = WiFi.softAPIP();       //get IP address
  //Serial.print("HotSpot IP: ");           //print IP address to serial monitor
  //Serial.println(myIP);


  Serial.println("");
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (mdns.begin("espWebSock", WiFi.localIP())) {
    Serial.println("MDNS responder started");
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }
  else {
    Serial.println("MDNS.begin failed");
  }
  Serial.print("Connect to http://espWebSock.local or http://");
  Serial.println(WiFi.localIP());


  

  server.on("/", handleRoot);                //define server startup behavior
  server.begin();                            //start server
  Serial.println("HTTP server started.");
  

  webSocket.begin();                         //start web socket
  webSocket.onEvent(webSocketEvent);

}

/**********************************************
 * This is where the controls happen - loops
 * continually. Most operations are managed
 * by the server.
 **********************************************/
void loop() {
  webSocket.loop();
  server.handleClient();
  
  /* Get a new sensor event */ 
  /*sensors_event_t accel, mag, gyro, temp;

  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  // print out accelleration data
  Serial.print("Accel X: "); Serial.print(accel.acceleration.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(accel.acceleration.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(accel.acceleration.z);     Serial.println("  \tm/s^2");

  // print out magnetometer data
  Serial.print("Magn. X: "); Serial.print(mag.magnetic.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(mag.magnetic.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(mag.magnetic.z);     Serial.println("  \tgauss");
  
  // print out gyroscopic data
  Serial.print("Gyro  X: "); Serial.print(gyro.gyro.x); Serial.print(" ");
  Serial.print("  \tY: "); Serial.print(gyro.gyro.y);       Serial.print(" ");
  Serial.print("  \tZ: "); Serial.print(gyro.gyro.z);     Serial.println("  \tdps");

  // print out temperature data
  Serial.print("Temp: "); Serial.print(temp.temperature); Serial.println(" *C");

  Serial.println("**********************\n");

  delay(250);
  */
}
