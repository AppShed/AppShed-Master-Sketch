/*
  This a simple example of the aREST Library for the ESP8266 WiFi chip.
  See the README file for more details.

  Written in 2015 by Marco Schwartz under a GPL license.
  Modified by Torsten Stauch, Dec 2016 (v24)

  This sketch combines the functionality of running a local Access Point, 
   or using the aREST Pro service.
   The default mode is "Local AP".
   To use the "aREST Pro Mode", the device must have pins D0 and D1 bridged at startup 
   (can remain bridged if those pins not used for anything else.)

  This sketch provides additional functions to support API calls from the AppShed app.js repo
   which can be found at https://github.com/AppShed/app.js 


  v 29 - Support for L9110S Motor Driver
  v 30 - Support for Motor Shield/Base Board. Motor Pins changed D1,2,3,4
  v 31 - aREST Pro support fixed
  v 32 - Remote Control fixed
  v 33 - AP/Pro bridge D6-D7
  v 34 - AP and Pro mode simultaneously
  v 35 - no bridging, always AP and Pro
  v 36 - Support for 2 types of Motor Drivers (Motor Shield, L298N)... todo: L9110
  v 37 - NeoPixel support 
  v 38 - HashMap added
  v 39 - /info shows all pins
  v 40 - don't start NeoPixel on setup
  v 41 - Removed pin update from loop, neet to call /pins to update pins, and /info to read values
  v 42 - Removed bug in loop
  
  --------------------------------------------------------
  NOTES
  --------------------------------------------------------

  from http://www.esp8266.com/viewtopic.php?f=40&t=6732
  Avoid using D3 with relays as it will latch on at boot and your wifi will not connect until you remove the GPIO connection from the relay.
  D8 Will also latch on and cause conflict do not use this. - See more at: http://www.esp8266.com/viewtopic.php?f=40&t=6732#sthash.IzOPRBbl.dpuf

  HASHMAP
  ref http://playground.arduino.cc/Code/HashMap


*/


// Import required libraries
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <aREST.h>
#include <Servo.h> 


// Variables to be exposed to the API
int build = 42;






// GPIO mapping
int gpio[] = {16,5,4,0,2,14,12,13,15};




// NeoPixel
#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif
Adafruit_NeoPixel strip1 = Adafruit_NeoPixel(24, gpio[5], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip2 = Adafruit_NeoPixel(24, gpio[6], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip3 = Adafruit_NeoPixel(24, gpio[7], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip4 = Adafruit_NeoPixel(24, gpio[8], NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel strip = strip1;




// aREST Pro key (that you can get at dashboard.arest.io)
char * key = "your_pro_key";
char * deviceName = "AppShed";



// Pin Values
int PinA0;
int PinD0;
int PinD1;
int PinD2;
int PinD3;
int PinD4;
int PinD5;
int PinD6;
int PinD7;
int PinD8;


// Clients
WiFiClient espClient;
PubSubClient client(espClient);

// Create aREST instance
aREST restAP = aREST();
aREST rest = aREST(client);


// Start the server
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);








// WiFi parameters
const char* ssid = "AppShed";
const char* password = "appshedrocks";
const char* ssidAP = deviceName;
const char* passwordAP = "appshedrocks";


// Functions
void callback(char* topic, byte* payload, unsigned int length);


// Variables to be exposed to the API
String analogValues = "";
String digitalValues = "";


// Declare functions to be exposed to the API
int calibrate(String command);
int commands(String command);
int logo(String command);
int runCommands(String command);
int attachServos(String command);
int setMotorDriver(String command);
int readPins(String command);

// create servo object to control a servo 
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;


// Other variables
int bridged = 0;
int maxPinsIndex = 8;
int maxCommandQueueIndex = 19;
int maxServosIndex = 3;
int logoQueueIndex = 0;
int maxLogoQueueIndex = 20;
int motorPinsConfigured = 0;
int maxPWM = 1023;
int secondsForWiFiConnection = 9;
int wifiConnected = 0;
unsigned long previousMillisWiFi = 0; 
const long intervalWiFi = 3000;  // check to see if WiFi connected ever x milliseconds while looping
unsigned long previousMillisPro = 0; 
const long intervalPro = 500;  // x milliseconds between Pro connections
unsigned long previousMillisPinRead = 0; 
const long intervalPinRead = 500;  // x milliseconds between updating pin state



int motorDriver = 0; // default motor driver. 
int pinA1A = 1; // default pin for left motor Forward
int pinA1B = 2; // left motor backwards
int pinB1A = 3; // right motor Forward
int pinB1B = 4; // right motor Backwards
//int pinLPWM = 4; // default pin for left motor PWM
//int pinRPWM = 5; // default pin for right motor PWM
int pinPen = 5; // default pin for Logo Pen
int leftPWMF = 1023; // default PWM value for the left motor
int rightPWMF = 1023; // default PWM value for the right motor
int leftPWMB = 1023; // default PWM value for the left motor
int rightPWMB = 1023; // default PWM value for the right motor
int speedFD = 100; // default forward speed in mm/second
int speedBK = 100; // default backward speed in mm/second
int speedLT = 180; // default left rotation speed in degrees/second
int speedRT = 180; // default right rotation speed in degrees/second
int speedARCFL = 90; // default forward left arc speed in degrees/second
int speedARCFR = 90; // default forward right arc speed in degrees/second
int speedARCBL = 90; // default back left arc speed in degrees/second
int speedARCBR = 90; // default back right arc speed in degrees/second


int currentLogoCommand[2]; // [command,value,endMillis]
unsigned long currentLogoCommandExpiry; 
int commandQueue[9][19][3]; // [pin 0-19][command# 0-max][0:format 1:value 2:duration]
int commandQueueIndex[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // the current index for each pin in commandQueue
unsigned long commandTimeout[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // the time at which the command for a pin times out
int logoQueue[20][2]; // [command#] [command,value]
int servoArray[4]; // the pin numbers for the servos
unsigned long lastRandom = 0;
int hashMapLength = 50; // size of our hashMap
int hashMapIndex = 0; // next index to use 
String hashMapKeys[50]; 
String hashMapValues[50]; 



void setup(void)
{
  // Start Serial
  Serial.begin(115200);

  Serial.println("");
  Serial.println("");
  Serial.println("");
  Serial.println("----------------------------------");
  Serial.print("Build: ");
  Serial.print(build);
  Serial.print("   Device: ");
  Serial.print(deviceName);
  Serial.println("");
  Serial.println("----------------------------------");


  // make sure PWM Range is set
  analogWriteRange(maxPWM);


  // Pin 4 is the default LED OUTPUT
  pinMode(gpio[4], OUTPUT);
  digitalWrite(gpio[4],0); // 0 turns the blue LED on


  // Allocate motor pins as OUTPUT and turn off
  pinMode(gpio[pinA1A], OUTPUT);
  pinMode(gpio[pinA1B], OUTPUT);
  pinMode(gpio[pinB1A], OUTPUT);
  pinMode(gpio[pinB1B], OUTPUT);  
//  pinMode(gpio[pinLPWM], OUTPUT);
//  pinMode(gpio[pinRPWM], OUTPUT);

  digitalWrite(gpio[pinA1A],0);
  digitalWrite(gpio[pinA1B],0);
  digitalWrite(gpio[pinB1A],0);
  digitalWrite(gpio[pinB1B],0);
  
  

    Serial.println("");
    Serial.println("-----------------------------");
    Serial.println("Mode: Pro");
    Serial.println("-----------------------------");


    // Set aREST key
    rest.setKey(key, client);
  
    // Set callback
    client.setCallback(callback);
  
    // Init variables and expose them to REST API
    rest.variable("build",&build);
    rest.variable("analogValues",&analogValues);
    rest.variable("digitalValues",&digitalValues);


    
    // Give name to device
    rest.set_name(deviceName);

      // Function to be exposed
    rest.function("calibrate",calibrate);
    rest.function("commands",commands);
    rest.function("logo",logo);
    rest.function("runCommands",runCommands);
    rest.function("attachServos",attachServos);
    rest.function("setMotorDriver",setMotorDriver);
    rest.function("readPins",readPins);

  
    // Connect to WiFi
    int delayMilli = 500;
    int countMilli = 0;
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
      delay(delayMilli);
      Serial.print(".");
      countMilli += delayMilli;
      if(countMilli/1000 > secondsForWiFiConnection){
        break;
      }
    }

    if(WiFi.status() == WL_CONNECTED){
      wifiConnected = 1;
      Serial.println("");
      Serial.println("WiFi connected");
      Serial.print("IP ");
      Serial.println(WiFi.localIP());
      delay(500);
    } else {
      // Make sure WiFi is disconnected
      WiFi.disconnect();
      delay(50);
      wifiConnected = 0;

      Serial.println("");
      Serial.println("*** WiFi NOT connected ***");
    }

    // Set output topic
    char* out_topic = rest.get_topic();

    

    
    Serial.println("");
    Serial.println("-----------------------------");
    Serial.println("Mode: Local AP");
    Serial.println("-----------------------------");

      
    // Init variables and expose them to REST API
    restAP.variable("build",&build); 
    restAP.variable("analogValues",&analogValues);
    restAP.variable("digitalValues",&digitalValues);


    
  
    // Function to be exposed
    restAP.function("calibrate",calibrate);
    restAP.function("commands",commands);
    restAP.function("logo",logo);
    restAP.function("runCommands",runCommands);
    restAP.function("attachServos",attachServos);
    restAP.function("setMotorDriver",setMotorDriver);
    restAP.function("readPins",readPins);


  
    // Give name & ID to the device (ID should be 6 characters long)
    restAP.set_id("local");
    restAP.set_name(deviceName);
  
  
    
    // Setup WiFi network
    WiFi.softAP(ssidAP, passwordAP);
    Serial.println("");
    Serial.println("WiFi Access Point created");
    Serial.print("SSID AP: ");
    Serial.println(ssidAP);
    Serial.print("Password: ");
    Serial.println(passwordAP);
    Serial.println("");
  


    server.begin();
    Serial.println("Server started");
  
    // Print the IP address
    IPAddress myIP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(myIP);

    delay(500);


    //NeoPixel
//    strip1.begin();
//    strip1.show(); // Initialize all pixels to 'off'
}








  





void loop() {

  unsigned long currentMillis = millis();



/*
 * NeoPixel Demo
  np_colorWipe(1,strip1.Color(255, 0, 0), 20); // Red
  np_colorWipe(1,strip1.Color(0, 255, 0), 20); // Green
  np_colorWipe(1,strip1.Color(0, 0, 255), 20); // Blue

  np_flash(1,strip1.Color(255, 255, 255),strip1.Color(0,0,0),20,10); 
*/


  // check if WiFi connected every so often

  if (currentMillis - previousMillisWiFi >= intervalWiFi) {
    // save the last time you checked
    previousMillisWiFi = currentMillis;

    if(wifiConnected == 1){
      if(WiFi.status() != WL_CONNECTED){
        Serial.println(WiFi.status());
        WiFi.disconnect();
        delay(50);
        wifiConnected = 0;
        Serial.println("");
        Serial.println("*** WiFi Connection lost... NOT connected ***");      
      }
    }
  }


  runLogo();
  runCommands("");


  
  

  if (wifiConnected && currentMillis - previousMillisPro >= intervalPro) {
    // save the last time you checked
    previousMillisPro = currentMillis;

    // Connect to the cloud
    rest.handle(client);

  }
  
  //else {
  {


      // Handle REST calls
      WiFiClient clientAP = server.available();
      if (!clientAP) {
        return;
      }
      while(!clientAP.available()){
        runLogo();
        runCommands("");
        delay(1);
      }
      restAP.handle(clientAP);    

  
  }


}







// Handles message arrived on subscribed topic(s)
void callback(char* topic, byte* payload, unsigned int length) {

  rest.handle_callback(client, topic, payload, length);

}



String getValue(String data, char separator, int index)
{
  int found = 0;
  int strIndex[] = {
    0, -1  };
  int maxIndex = data.length()-1;
  for(int i=0; i<=maxIndex && found<=index; i++){
    if(data.charAt(i)==separator || i==maxIndex){
      found++;
      strIndex[0] = strIndex[1]+1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }
  String rVal = "";
  if(found>index){
    rVal = data.substring(strIndex[0], strIndex[1]);
    rVal.replace(" ","");
  }
  
  return rVal;
}





// Custom function accessible by the API
int readPins(String command) {
  // update the saved pin values for specified pins
  // command containts a comma separated list of the pins to read
  // e.g. commands = "A0,D3,D5"

  bool readAll = false;
  
  if(command == "")
    readAll = true;

  int wait = 1;

  // add a leading and trailing comma to make inspection easier
  command = ","+command+",";

  String str_Command;
  int item_counter = 0;


  while(1){
    str_Command = getValue(command, ',', item_counter++);

  
    if(!readAll && str_Command == "")
      break;

    if(readAll || str_Command == "A0"){
      PinA0 = analogRead(A0);
      delay(wait);
    }
    if(readAll || str_Command == "D0"){
      PinD0 = digitalRead(gpio[0]);
      delay(wait);
    }
    if(readAll || str_Command == "D1"){
      PinD1 = digitalRead(gpio[1]);
      delay(wait);
    }
    if(readAll || str_Command == "D2"){
      PinD2 = digitalRead(gpio[2]);
      delay(wait);
    }
    if(readAll || str_Command == "D3"){
      PinD3 = digitalRead(gpio[3]);
      delay(wait);
    }
    if(readAll || str_Command == "D4"){
      PinD4 = digitalRead(gpio[4]);
      delay(wait);
    }
    if(readAll || str_Command == "D5"){
      PinD5 = digitalRead(gpio[5]);
      delay(wait);
    }
    if(readAll || str_Command == "D6"){
      PinD6 = digitalRead(gpio[6]);
      delay(wait);
    }
    if(readAll || str_Command == "D7"){
      PinD7 = digitalRead(gpio[7]);
      delay(wait);
    }
    if(readAll || str_Command == "D8"){
      PinD8 = digitalRead(gpio[8]);
      delay(wait);
    }

    break;    

  }
  
  analogValues = String(PinA0);
  digitalValues = String(PinD0) + "," + String(PinD1) + "," + String(PinD2) + "," + String(PinD3) + "," + String(PinD4) + "," + String(PinD5) + "," + String(PinD6) + "," + String(PinD7) + "," + String(PinD8);

  return 1;
}



// Custom function accessible by the API
int commands(String command) {

  // Commands will be sent in this format:
  // format,pin,value,duration:format,pin,value,duration:....
  // format is 
  //  0-analog 
  //  1-digital
  //  101 - 199 -NeoPixel


  String str_Command;
  int item_counter = 0;

  while(1){
    str_Command = getValue(command, ':', item_counter++);
    if(str_Command == "")
      break;


    addCommandToQueue(str_Command);

  }
  return item_counter;
}





void addCommandToQueue(String command){

  // expecting command format,pin,value,duration

  String data;
  int formatInt;
  int pinInt;
  int valueInt;
  String value = "";
  int durationInt;
  
  int item_counter = 0;

  while(1){
    data = getValue(command, ',', item_counter++);
    if(data == "")
      break;
    formatInt = data.toInt();
    
    data = getValue(command, ',', item_counter++);
    if(data == "")
      break;
    pinInt = data.toInt();
    if(pinInt < 0 || pinInt > maxPinsIndex)
      break;
    
    data = getValue(command, ',', item_counter++);
    if(data == "")
      break;

    // If this format sends a string value, save it in the hashMap
    if(hasStringValue(formatInt)){
      value = data; // Also save the string value
      valueInt = getRandom(1000,2000);
      setHashValue(String(valueInt),value);


    }else{
      valueInt = data.toInt();
    }
        
    data = getValue(command, ',', item_counter++);
    // default duration is 0 (if ""), i.e. indefinite duration
    durationInt = data.toInt();

    int commandIndex = commandQueueIndex[pinInt];


    // If the queue is full, this command is lost
    if(commandIndex > maxCommandQueueIndex)
      break;
      
    commandQueue[pinInt][commandIndex][0] = formatInt;
    commandQueue[pinInt][commandIndex][1] = valueInt;
    commandQueue[pinInt][commandIndex][2] = durationInt;

    // increment the index, for the next command to use
    commandQueueIndex[pinInt]++;

    break;
  }

}


int runLogo(){

  // Run any pending Logo commands

  unsigned long currentMillis = millis();

  // Don't do anything if the current command hasn't expired
  if(currentLogoCommand[0] >= 0 && currentLogoCommandExpiry > currentMillis)
    return 1;
    
  else {
    // No current command OR Current command has expired

    // If there's a command in the queue, start it
    if(logoQueueIndex > 0){
      // save the new command details
      int new_command = logoQueue[0][0];
      int new_value = logoQueue[0][1];


      // remove the command from the Queue
      // move each command down one index (Command at 0 will be overwritten)
      for(int pos=0;pos<(logoQueueIndex-1);pos++){
        logoQueue[pos][0] = logoQueue[(pos+1)][0];
        logoQueue[pos][1] = logoQueue[(pos+1)][1];
      }
      // make sure the last one is removed (set to -1)
      logoQueue[(logoQueueIndex-1)][0] = -1;
      logoQueue[(logoQueueIndex-1)][1] = -1;
     
      // decrement the queue index
      logoQueueIndex--;
      
      // execute the new command
      executeLogo(new_command,new_value);
      
    } else{
      // No command in the queue

      // If this is an indefinite command, do nothing
      if(currentLogoCommandExpiry == 0)
        return 1;
      else if(currentLogoCommand[0] == 0)
        return 1; // current command is STOP, so don't re-execute
      else{
        // else stop the command
        executeLogo(0);      
      }
      
    }
    
  }
}


int executeLogo(int command){
  return executeLogo(command,0);
}


int executeLogo(int command, int value){

  // get the duration for this command
  int duration = getLogoDuration(command,value);
  unsigned long currentMillis = millis();

  // save the command to the currentLogoCommand
  currentLogoCommand[0] = command;
  currentLogoCommand[1] = value;
  currentLogoCommandExpiry = (currentMillis + (long)duration);
  // Special case for indefinite commands
  if(value == 0)
    currentLogoCommandExpiry = 0;


  // Make sure Motor Pins have been configured
  if(motorPinsConfigured == 0){
    initialiseMotorPins();
    pinMode(gpio[pinPen], OUTPUT);
    digitalWrite(gpio[pinPen],1); // Put the pen down by default.

  }



  // set the required pins
  // 0.   ST x (or STOP) - Stop moving for x milliseconds
  if(command == 0){
    motorLeftST();
    motorRightST();
  }

    // 1.   FD x (or FORWARD) - Move forward x millimeters, Example: FD 100
  if(command == 1){
    motorLeftFD();
    motorRightFD();
  }
    // 2.   BK x (or BACK) - Move Backward x millimeters, Example: BK 100
  if(command == 2){    
    motorLeftBK();
    motorRightBK();
  }
    // 3.   LT x (or LEFT) - Rotate the turtle x degrees left, Example: LT 45
  if(command == 3){
    motorLeftBK();
    motorRightFD();
  }
    // 4.   RT x (or RIGHT) - Rotate the turtle x degrees right, Example: RT 45
  if(command == 4){
    motorLeftFD();
    motorRightBK();
  }
    // 5.   ARCFL x Move in an arc going forward and left for x degrees, Example: ARCFL 45
  if(command == 5){
    motorLeftST();
    motorRightFD();
  }
    // 6.   ARCFR x Move in an arc going forward and right for x degrees, Example: ARCFR 45
  if(command == 6){
    motorRightST();
    motorLeftFD();
  }
    // 7.   ARCBL x Move in an arc going backward and left for x degrees, Example: ARCBL 45
  if(command == 7){
    motorLeftST();
    motorRightBK();
  }
    // 8.   ARCBR x Move in an arc going backward and right for x degrees, Example: ARCBR 45
  if(command == 8){
    motorRightST();
    motorLeftBK();
  }


  
  
    // 9.   PU (or PENUP) - Lift the Pen
  if(command == 9){
    // Pen Up. Default behaviour turns the pinPen on and off
    digitalWrite(gpio[pinPen],0);    
  }
    // 10.  PD (or PENDOWN) - Put the Pen down
  if(command == 10){
    // Pen Down. Default behaviour turns the pinPen on and off. Other Option to move servo
    digitalWrite(gpio[pinPen],1);
  }

            
  return 1;
}


void motorLeftST(){
  if(motorDriver == 1){
    analogWrite(gpio[pinA1A],0);    
    analogWrite(gpio[pinA1B],0);    
    digitalWrite(gpio[pinA1A],0);    
    digitalWrite(gpio[pinA1B],0);    

  } else if(motorDriver == 2){
    
  } else {
      analogWrite(gpio[pinA1A],0);
      digitalWrite(gpio[pinB1A],0);
  }
}

void motorRightST(){
  if(motorDriver == 1){
    analogWrite(gpio[pinB1A],0);    
    analogWrite(gpio[pinB1B],0);    
    digitalWrite(gpio[pinB1A],0);    
    digitalWrite(gpio[pinB1B],0);        
  } else if(motorDriver == 2){
    
  } else {
      analogWrite(gpio[pinA1B],0);
      digitalWrite(gpio[pinB1B],0);
  }
}

void motorLeftFD(){
  if(motorDriver == 1){
    digitalWrite(gpio[pinA1A],1);
    analogWrite(gpio[pinA1B],(maxPWM - leftPWMF));
  } else if(motorDriver == 2){
    
  } else {
    analogWrite(gpio[pinA1A],leftPWMF);
    digitalWrite(gpio[pinB1A],1);
  }
}

void motorRightFD(){
  if(motorDriver == 1){
    digitalWrite(gpio[pinB1A],1);
    analogWrite(gpio[pinB1B],(maxPWM - rightPWMF));
  } else if(motorDriver == 2){
    
  } else {
    analogWrite(gpio[pinA1B],rightPWMF);
    digitalWrite(gpio[pinB1B],1);    
  }
}


void motorLeftBK(){
  if(motorDriver == 1){
    digitalWrite(gpio[pinA1A],0);    
    analogWrite(gpio[pinA1B],leftPWMB);
  } else if(motorDriver == 2){
    
  } else {
    analogWrite(gpio[pinA1A],leftPWMF);
    digitalWrite(gpio[pinB1A],0);
  }
}

void motorRightBK(){
  if(motorDriver == 1){
    digitalWrite(gpio[pinB1A],0);    
    analogWrite(gpio[pinB1B],rightPWMB);
  } else if(motorDriver == 2){
    
  } else {
    analogWrite(gpio[pinA1B],rightPWMF);
    digitalWrite(gpio[pinB1B],0);    
  }
}

void initialiseMotorPins(){
  // Configure the motor pins as outputs
  
  // turn pin 4 off
  pinMode(gpio[pinA1A], OUTPUT);
  pinMode(gpio[pinA1B], OUTPUT);
  pinMode(gpio[pinB1A], OUTPUT);
  pinMode(gpio[pinB1B], OUTPUT);
//  pinMode(gpio[pinLPWM], OUTPUT);
//  pinMode(gpio[pinRPWM], OUTPUT);

  motorPinsConfigured = 1;
}

  

int getLogoDuration(int command, int value){
  // return the number of milliseconds for this command.

  // Special case: 0
  if(value == 0)
    return 0;

  if(command == 0)
    return value;
  if(command == 1)
    return (int)(1000.0*(float)value/(float)speedFD);
  if(command == 2)
    return (int)(1000.0*(float)value/(float)speedBK);
  if(command == 3)
    return (int)(1000.0*(float)value/(float)speedLT);
  if(command == 4)
    return (int)(1000.0*(float)value/(float)speedRT);
  if(command == 5)
    return (int)(1000.0*(float)value/(float)speedARCFL);
  if(command == 6)
    return (int)(1000.0*(float)value/(float)speedARCFR);
  if(command == 7)
    return (int)(1000.0*(float)value/(float)speedARCBL);
  if(command == 8)
    return (int)(1000.0*(float)value/(float)speedARCBR);
    
  return 0;

}


int calibrate(String command){
  // set calibration constants
  // command: constant1,val1;const2,val2;...


  String str_Pair;
  String str_Constant;
  String str_Value;
  
  int item_counter = 0;
  while(1){
    // Get the next Pair from the command string
    str_Pair = getValue(command, ';', item_counter++);
    if(str_Pair == "")
      break;

    // Break it into Command,Value
    while(1){
      str_Constant = getValue(str_Pair, ',', 0);
      if(str_Constant == "")
        break;
      str_Value = getValue(str_Pair, ',', 1);
      if(str_Value == "")
        break;
      int iValue = str_Value.toInt();

      updateConstant(str_Constant,iValue);
      break;
      
    }


  }
  return item_counter;
  
}


int updateConstant(String c, int value){
  // Update the 'constant' c setting it to value

  if(c == "leftPWMF")
    leftPWMF = value;
  if(c == "rightPWMF")
    rightPWMF = value;
  if(c == "leftPWMB")
    leftPWMB = value;
  if(c == "rightPWMB")
    rightPWMB = value;
  if(c == "speedFD")
    speedFD = value;
  if(c == "speedBK")
    speedBK = value;
  if(c == "speedLT")
    speedLT = value;
  if(c == "speedRT")
    speedRT = value;
  if(c == "speedARCFL")
    speedARCFL = value;
  if(c == "speedARCFR")
    speedARCFR = value;
  if(c == "speedARCBL")
    speedARCBL = value;
  if(c == "speedARCBL")
    speedARCBL = value;
  if(c == "speedARCBR")
    speedARCBR = value;


  return 1;
}


int runCommands(String command){

  // format: 
  //  0: analog (pwm)
  //  1: digital
  //  101-199: NeoPixel

  for(int pinInt=0;pinInt<maxPinsIndex;pinInt++){
    int nextCommandIndex = commandQueueIndex[pinInt];
    if(nextCommandIndex>0){

      // if the timeout has expired, run the next command (index 0)
      unsigned long currentMillis = millis();
      if(commandTimeout[pinInt] < currentMillis){


        int format = commandQueue[pinInt][0][0];
        int value = commandQueue[pinInt][0][1];
        int duration = commandQueue[pinInt][0][2];

        // set the timeout. If duration is 0, then no timeout
        if(duration == 0)
          commandTimeout[pinInt] = 0;
        else
          commandTimeout[pinInt] = (currentMillis + duration);
        
        // move each command down one index (Command at 0 will be overwritten)
        for(int pos=0;pos<(nextCommandIndex-1);pos++){
          commandQueue[pinInt][pos][0] = commandQueue[pinInt][(pos+1)][0];
          commandQueue[pinInt][pos][1] = commandQueue[pinInt][(pos+1)][1];
          commandQueue[pinInt][pos][2] = commandQueue[pinInt][(pos+1)][2];
        }
        // make sure the last one is not duplicated
        commandQueue[pinInt][(nextCommandIndex-1)][0] = -1;
        commandQueue[pinInt][(nextCommandIndex-1)][1] = -1;
        commandQueue[pinInt][(nextCommandIndex-1)][2] = -1;
        // decrement the commandQueueIndex - because we have removed one of the commands
        commandQueueIndex[pinInt]--;
          
        if(format == 0){
          // Analog
          
          pinMode(gpio[pinInt], OUTPUT);
          // special case if the pin has a servo attached
          if(servoArray[0] == pinInt && servo1.attached())
            servo1.write(value);
          else if(servoArray[1] == pinInt && servo2.attached())
            servo2.write(value);
          else if(servoArray[2] == pinInt && servo3.attached())
            servo3.write(value);
          else if(servoArray[3] == pinInt && servo4.attached())
            servo4.write(value);
          else  
            analogWrite(gpio[pinInt],value);
        
        } else if(format == 1){
          // Digital
          
          pinMode(gpio[pinInt], OUTPUT);
          digitalWrite(gpio[pinInt],value);
        
        } else if(format >= 101 && format <= 199){
          // NeoPixel
        
          neoPixelRoutine(format,pinInt,value);
        }
      }
      
    }
    
  }
  return 1;
}





int attachServos(String command){

  // attach a servo to a pin
  // expecting command servoNumber,pin:servoNumber,pin...


  String str_Command;
  int item_counter = 0;
  int item_counter2 = 0;
  String data;
  int servoNumber = 0;
  int pinInt;

  while(1){
    str_Command = getValue(command, ':', item_counter++);
    if(str_Command == "")
      break;
  
    while(1){
      data = getValue(command, ',', item_counter2++);
      if(data == "")
        break;
      servoNumber = data.toInt();
      
      data = getValue(command, ',', item_counter2++);
      if(data == "")
        break;
      pinInt = data.toInt();
      if(pinInt < 0 || pinInt > maxPinsIndex)
        break;

      
      if(servoNumber == 1){
        if(servo1.attached())servo1.detach();
        servo1.attach(gpio[pinInt]);
        servoArray[0] = pinInt;
      }
      if(servoNumber == 2){
        if(servo2.attached())servo2.detach();
        servo2.attach(gpio[pinInt]);
        servoArray[1] = pinInt;
      }
      if(servoNumber == 3){
        if(servo3.attached())servo3.detach();
        servo3.attach(gpio[pinInt]);
        servoArray[2] = pinInt;
      }
      if(servoNumber == 4){
        if(servo4.attached())servo4.detach();
        servo4.attach(gpio[pinInt]);
        servoArray[3] = pinInt;
      }
      
    }
  }  

  return item_counter;
}




int attachNeoPixel(String command){

  // attach a NeoPixel to a pin
  // expecting command neoPixelNumber,pin:neoPixelNumber,pin...

  // to be implemented... use same code as attacheServo
  

  return 1;
}



int setMotorDriver(String command){
  // set the motorDriver variable to ensure the correct pin settings are made according to the driver specifications
  // returns the value that motorDriver was set to
  
  // motorDriver 0 - NodeMCU Motor Shield - A1A PWM A1B Digital FD 80:1, BK 80:0
  // motorDriver 1 - L298N - A1A PWM or digital, A1B PWM or Digital, FD 80:0, BK 0:80
  // motorDriver 2 - L9110 - A1A PWM A1B Digital FD 20:1 (will go FD 80%) BK 80:0  


  int type = command.toInt();
  if(type >= 0 && type <= 2){
    motorDriver = type;
    
    return motorDriver;
  }

  return -1;
}


// Custom function accessible by the API
int logo(String command) {
    
  command.replace(" /"," ");

  // Drives two motors in various directions using Logo commands
  // Default pins for left and right motors are set by variables leftF, leftB, rightF, rightB
  // commands will be sent in this format:
  // command,[value];command,[value];<repeat>
  // Commands can be: 
  // 0.   ST x (or STOP) - Stop moving for x milliseconds
  // 1.   FD x (or FORWARD) - Move forward x millimeters, Example: FD 100
  // 2.   BK x (or BACK) - Move Backward x millimeters, Example: BK 100
  // 3.   LT x (or LEFT) - Rotate the turtle x degrees left, Example: LT 45
  // 4.   RT x (or RIGHT) - Rotate the turtle x degrees right, Example: RT 45
  // 5.   ARCFL x Move in an arc going forward and left for x degrees, Example: ARCFL 45
  // 6.   ARCFR x Move in an arc going forward and right for x degrees, Example: ARCFR 45
  // 7.   ARCBL x Move in an arc going backward and left for x degrees, Example: ARCBL 45
  // 8.   ARCBR x Move in an arc going backward and right for x degrees, Example: ARCBR 45
  // 9.   PU (or PENUP) - Lift the Pen
  // 10.  PD (or PENDOWN) - Put the Pen down

  // In most cases x is optional and defaults to 0. This will result in <command> running indefinite, until the next command is sent.

  String str_CommandValue;
  String str_Command;
  String str_Value;
  
  int item_counter = 0;
  while(1){
    // Get the next command,value from the command string
    str_CommandValue = getValue(command, ';', item_counter++);
    if(str_CommandValue == "")
      break;

    // Break it into Command,Value
    while(1){
      str_Command = getValue(str_CommandValue, ',', 0);
      if(str_Command == "")
        break;
      str_Value = getValue(str_CommandValue, ',', 1);
      int iValue = 0; // default value is 0
      if(str_Value > "")
        iValue = str_Value.toInt();

      addLogoToQueue(str_Command,iValue);
      break;
      
    }


  }
  return item_counter;
}





int addLogoToQueue(String command, int value){

  // If we're at capacity, ... do something to save this data and come back to it later
  // to be implemented
  if(logoQueueIndex > maxLogoQueueIndex)
    return 0;

  // Case insensitive
  command.toUpperCase();

  int int_command;
  
  // 0.   ST (or STOP) - Stop moving
  // 1.   FD x (or FORWARD) - Move forward x millimeters, Example: FD 100
  // 2.   BK x (or BACK) - Move Backward x millimeters, Example: BK 100
  // 3.   LT x (or LEFT) - Rotate the turtle x degrees left, Example: LT 45
  // 4.   RT x (or RIGHT) - Rotate the turtle x degrees right, Example: RT 45
  // 5.   ARCFL x Move in an arc going forward and left for x degrees, Example: ARCFL 45
  // 6.   ARCFR x Move in an arc going forward and right for x degrees, Example: ARCFR 45
  // 7.   ARCBL x Move in an arc going backward and left for x degrees, Example: ARCBL 45
  // 8.   ARCBR x Move in an arc going backward and right for x degrees, Example: ARCBR 45
  // 9.   PU Lift the Pen Up, Example: PU
  // 10.  Put the Pen Down, Example: PD

  if(command == "ST" || command == "STOP")
    int_command = 0;
  else if(command == "FD" || command == "FORWARD")
    int_command = 1;
  else if(command == "BK" || command == "BACK")
    int_command = 2;
  else if(command == "LT" || command == "LEFT")
    int_command = 3;
  else if(command == "RT" || command == "RIGHT")
    int_command = 4;
  else if(command == "ARCFL")
    int_command = 5;
  else if(command == "ARCFR")
    int_command = 6;
  else if(command == "ARCBL")
    int_command = 7;
  else if(command == "ARCBR")
    int_command = 8;
  else if(command == "PU" || command == "PENUP")
    int_command = 9;
  else if(command == "PD" || command == "PENDOWN")
    int_command = 10;
  else 
    return 0; // command not supported     
    
  logoQueue[logoQueueIndex][0] = int_command;
  logoQueue[logoQueueIndex][1] = value;

  logoQueueIndex++;
  
  return 1;
}














/*
 * ==========================================================
 * ================ = N E O   P I X E L  ====================
 * ==========================================================
 */


void neoPixelRoutine(int routine, int s, int value){

  
  // run a NeoPixel routine on NeoPixel Strip `s` passing value
  // routine can be:
  //  101: clear - set all dots off
  //  102: setColor - sets the color of the whole NeoPixel to value. 
  //      Expects value to be an RGB Integer
  //  105: 
  // value is the key in the hashMap
  

  String params = String(value);
  String strRoutine = params.substring(0,2);
  String strWait = params.substring(2,4);
  String strColor = params.substring(6,20); 



  if(routine == 101)
    np_clear(s);
  
  if(routine == 102)
    np_setColor(s,getHashValue(value).toInt());

//  if(routine == 105)
//    np_colorWipe(pin,getHashValue(value).toInt());

}






// Routine 101 - clear
// Set all dots to off
int np_clear(int s) {
  return np_setColor(s,0);
}


// Routine 102 - setColor 
int np_setColor(int s, uint32_t c) {
  // Set all dots to a color
  // s - strip number
  // c - color

  if(s==1){strip = strip1;} else if(s==2){strip = strip2;} else if(s==3){strip = strip3;} else if(s==4){strip = strip4;} else return 0;

  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
  }
  strip.show();
  return 1;
}



// Routine 104 - flash 
int np_flash(int s, uint32_t c1, uint32_t c2, uint8_t wait, int counter) {
  // Flashes between two colors
  // s - strip number, 1-4
  // c1 - color 1
  // c2 - color 2
  // wait - time to wait between color changes
  // counter - how many times to flash

  if(s==1){strip = strip1;} else if(s==2){strip = strip2;} else if(s==3){strip = strip3;} else if(s==4){strip = strip4;} else return 0;

  for(int a=0; a<counter; a++) {
    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c1);
    }
    strip.show();
    delay(wait);

    for(uint16_t i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, c2);
    }
    strip.show();
    delay(wait);
  }
  return 1;
}



// Routine 105 - Color Wipe
// Fill the dots one after the other with a color
int np_colorWipe(int s, uint32_t c, uint8_t wait) {

  if(s==1){strip = strip1;} else if(s==2){strip = strip2;} else if(s==3){strip = strip3;} else if(s==4){strip = strip4;} else return 0;
    
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }

  return 1;
}

void np_rainbow(Adafruit_NeoPixel st, uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256; j++) {
    for(i=0; i<strip.numPixels(); i++) {
      strip.setPixelColor(i, np_Wheel(st,(i+j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

// Slightly different, this makes the rainbow equally distributed throughout
void np_rainbowCycle(Adafruit_NeoPixel st, uint8_t wait) {
  uint16_t i, j;

  for(j=0; j<256*5; j++) { // 5 cycles of all colors on wheel
    for(i=0; i< strip.numPixels(); i++) {
      strip.setPixelColor(i, np_Wheel(st,((i * 256 / strip.numPixels()) + j) & 255));
    }
    strip.show();
    delay(wait);
  }
}

//Theatre-style crawling lights.
void np_theaterChase(Adafruit_NeoPixel st, uint32_t c, uint8_t wait) {
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

//Theatre-style crawling lights with rainbow effect
void np_theaterChaseRainbow(Adafruit_NeoPixel st, uint8_t wait) {
  for (int j=0; j < 256; j++) {     // cycle all 256 colors in the wheel
    for (int q=0; q < 3; q++) {
      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, np_Wheel(st, (i+j) % 255));    //turn every third pixel on
      }
      strip.show();

      delay(wait);

      for (uint16_t i=0; i < strip.numPixels(); i=i+3) {
        strip.setPixelColor(i+q, 0);        //turn every third pixel off
      }
    }
  }
}

// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t np_Wheel(Adafruit_NeoPixel st, byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return st.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return st.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return st.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}

















/*
 * ==========================================================
 * =========================== Hash Map  ====================
 * ==========================================================
 */



String getHashValue(int key){
  return getHashValue(String(key));
}

String getHashValue(String key){

  for (int i=0; i < hashMapIndex; i++) {
    if(hashMapKeys[i] == key){
      return hashMapValues[i];
    }
  }
  
  return "";
}


void setHashValue(String key, String value){
  int done = 0;
  for (int i=0; i < hashMapIndex; i++) {
    if(hashMapKeys[i] == key){
      hashMapValues[i] = value;
      done = 1;
    }
  }

  if(done == 0){
    hashMapKeys[hashMapIndex] = key;
    hashMapValues[hashMapIndex] = value;
    hashMapIndex ++;
  }

}

int getRandom(int from,int to){
  randomSeed(lastRandom);
  lastRandom = (int) random(from,to);
  return lastRandom;
}

int hasStringValue(int formatInt){
  // if this command format uses a string value, return true. 
  // this is the case where the format is neoPixel and other formats that pass a CSV as the value, not a number

  if(formatInt > 100)
    return 1;

  return 0;
}














// Fill the dots one after the other with a color
void colorWipe(uint32_t c, uint8_t wait) {
  for(uint16_t i=0; i<strip.numPixels(); i++) {
    strip.setPixelColor(i, c);
    strip.show();
    delay(wait);
  }
}


// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  WheelPos = 255 - WheelPos;
  if(WheelPos < 85) {
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  }
  if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
  WheelPos -= 170;
  return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
}
