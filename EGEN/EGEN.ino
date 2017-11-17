/**************************************************************
 *  This program contains the microcontroller program and the GUI
 *  in one. The GUI is setup through the header file "index.h"
 *  in HTML5 and CSS3 using a web socket. This enables data to
 *  flow both directions. The microcontroller hosts the website
 *  and a client can connect and send data to the microcontroller.
 *  The microcontroller sends data as well, such as the IMU
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
Adafruit_MotorShield AFMS = Adafruit_MotorShield();                                 //create motor shield object with default I2C address and motor object 
Adafruit_DCMotor *frontMotor = AFMS.getMotor(3);                                    //get both motors and initialize variables with them
Adafruit_DCMotor *backMotor = AFMS.getMotor(2);                                       
Servo servo;                                                                        //get servo

Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);                                      // Use I2C, ID #1000 for the IMU sensor



/**********************************************
 * Define specs for internet connection
 **********************************************/
static const char* ssid = "Droopy Rhino";
static const char* password = "passwordE10";
MDNSResponder mdns;

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
const char IMU[] = "IMU";
int i = 0;
int j = 75;
float lowAcc = 500;
float highAcc = 0;
float lowMag = 500;
float highMag = -500;
float lowGyro = 500;
float highGyro = -500;
float lowTemp = 100;
float highTemp = 0;

/**********************************************
 * Define functions - no prototypes since they
 * are implemented here
 **********************************************/
void configureSensor(void)                                                             //function to setup the IMU sensor
{
  // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_16G);
  
  // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS0_MAGGAIN_12GAUSS);

  // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_2000DPS);
}


//display details about the sensor to serial (only for if wired)
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
        if (i >= -40 && i < 40) {
          i = 40;
        }
        int f = i + 20;
        if (f < 255) {
          i += 20;
        }
        else i = 255;
        motion(i);
      }
      if (strcmp(BACK, (const char *)payload) == 0) {                           //case for if backward button was pressed
        if (i <= 40 && i > -40) {
          i = -40;
        }
        else if (i <= 40 && i > 0) {
          i = 0;
        }
        int f = i - 20;
        if (f > -255) {
          i -= 20;
        }
        else i = -255;
        motion(i);
      }
      if (strcmp(RIGHT, (const char *)payload) == 0) {                          //case for if right button was pressed
        if (j > 95) {
          j = 95;
        }
        else j = 70;
        turn(j);
      }
      if (strcmp(LEFT, (const char *)payload) == 0) {                           //case for if left button was pressed
        if (j < 95) {
          j = 95;
        }
        else j = 120;
        turn(j);
      }
      if (strcmp(STOP, (const char *)payload) == 0) {                           //case for if left button was pressed
        quit();
      }
      if (strcmp(IMU, (const char *)payload) == 0) {                           //case for if IMU page button was pressed
        handleIMU();
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

void handleIMU() {                                                                     //prints IMU data to page
  //Get a new sensor event 
  sensors_event_t accel, mag, gyro, temp;
  lsm.getEvent(&accel, &mag, &gyro, &temp); 

  //send the IMU data to GUI
  String html = "<!doctype html><html><head><META HTTP-EQUIV=\"refresh\" CONTENT=\"1\"><title>IMU Data</title><style>body{background-color:#333;}h2{font-size:40px;color:white;}h3{font-size:36px;color:white;}p{font-size:32px;color:white;}</style></head><body>";
  // print out IMU data
  float acc = sqrt(sq(accel.acceleration.x) + sq(accel.acceleration.y) + sq(accel.acceleration.z)) - 9;
  float magno = sqrt(sq(mag.magnetic.x) + sq(mag.magnetic.y) + sq(mag.magnetic.z));
  float gy = sqrt(sq(gyro.gyro.x) + sq(gyro.gyro.y) + sq(gyro.gyro.z)) - 5;
  float temper = temp.temperature;
  if (acc < lowAcc) {
    lowAcc = acc;
  }
  if (acc > highAcc) {                                                                //these all initialize the variables to keep track of extreme values
    highAcc = acc;
  }
  if (magno < lowMag) {
    lowMag = magno;
  }
  if (magno > highMag) {
    highMag = magno;
  }
  if (gy < lowGyro) {
    lowGyro = gy;
  }
  if (gy > highGyro) {
    highGyro = gy;
  }
  if (temper < lowTemp) {
    lowTemp = temper;
  }
  if (temper > highTemp) {
    highTemp = temper;
  }
  String inf1 = "<h2>Current Readings:</h2><p>Acceleration: " + String(acc) + " m/s^2<br>";
  String inf2 = "Magnetometer: " + String(magno) + " rad/s<br>";
  String inf3 = "Gyroscope: " + String(gy) + " dps<br>";
  String inf4 = "Temperature: " + String(temper) + " *C<br><br><br></p><h3>Extremes:</h3><p>Acceleration:<br>";
  String maxMin = "Max: " + String(highAcc) + "<br>Min: " + String(lowAcc)+ "<br><br>Magnetometer:<br>Max: " + String(highMag) + "<br>Min: " + String(lowMag) + "<br><br>Gryoscope:<br>Max: " + String(highGyro) + "<br>Min: " + String(lowGyro) + "<br><br>Temperature:<br>Max: " + String(highTemp) + "<br>Min: " + String(lowTemp);
  String htmlEnd = "</p></body></html>";
  String info = html + inf1 + inf2 + inf3 + inf4 + maxMin + htmlEnd;
  server.send(200, "text/html", info);  
}

void handleLED() {                                                                     //function to turn LED on and off
  digitalWrite(LED_BUILTIN,!digitalRead(LED_BUILTIN));                                 //sets the state of the LED to the inverse of what it currently was)
}

void motion(int d) {                                                                   //function for moving the car
  if (d > -10) {
    backMotor->run(FORWARD);
    frontMotor->run(FORWARD);
  }
  else if (d < 10) {
    backMotor->run(BACKWARD);
    frontMotor->run(BACKWARD);
  }
  backMotor->setSpeed(abs(d));
  frontMotor->setSpeed(abs(d));
}

void turn(int k) {                                                                     //function for turning the car
  servo.write(k);
}

void quit() {                                                                          //function to stop the car's motion
  i = 0;
  backMotor->setSpeed(0);
  frontMotor->setSpeed(0);
}



/**********************************************
 * Function to setup the program before loop
 **********************************************/
void setup() {
  Serial.begin(115200);                      //begin serial transmissions on 115200 baud
  delay(100);
  Serial.println("");    
  pinMode(LED_BUILTIN, OUTPUT);              //initialize LED, set it to off
  digitalWrite(LED_BUILTIN, HIGH); 
  AFMS.begin();                              //create motorshield with the default frequency 1.6KHz

  frontMotor->setSpeed(150);                 //Set the speed to start, from 0 (off) to 255 (max speed)
  frontMotor->run(FORWARD);                  //turn on motor
  frontMotor->run(RELEASE);
  backMotor->setSpeed(150);                
  backMotor->run(FORWARD);                  
  backMotor->run(RELEASE);
  servo.attach(2);                           //get servo from I/O pin 2 and initialize it's degree (82 degrees seemed to be centered)
  servo.write(92);

  
  if(!lsm.begin())
  {
    //There was a problem detecting the LSM9DS0
    Serial.print(F("No LSM9DS0 detected."));
  }
  else {
    Serial.println(F("Found LSM9DS0 9DOF"));
    //Display some basic information on this sensor 
    displaySensorDetails();
      
    //Setup the sensor gain and integration time 
    configureSensor();
    Serial.println("");
  }
  
  WiFi.begin(ssid, password);
  
  WiFi.mode(WIFI_AP);                     //define wifi as access point
  WiFi.softAP(ssid, password);            //initialize web server
  IPAddress myIP = WiFi.softAPIP();       //get IP address
  Serial.print("\nHotSpot IP: ");         //print IP address to serial monitor
  Serial.println(myIP);
  Serial.println("");

  if (mdns.begin("espWebSock", WiFi.localIP())) {
    Serial.println("MDNS responder started");
    mdns.addService("http", "tcp", 80);
    mdns.addService("ws", "tcp", 81);
  }
  else {
    Serial.println("MDNS.begin failed");
  }
  

  server.on("/", handleRoot);                //define server startup behavior
  server.on("/IMU", handleIMU);              //define behavior on the /IMU page
  server.begin();                                      //start server
  Serial.println("HTTP server started.");
  

  webSocket.begin();                                   //start web socket
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
  WiFiClient client;
}
