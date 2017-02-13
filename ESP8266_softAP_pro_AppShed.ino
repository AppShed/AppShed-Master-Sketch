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
  
  --------------------------------------------------------
  NOTES
  --------------------------------------------------------

  from http://www.esp8266.com/viewtopic.php?f=40&t=6732
  Avoid using D3 with relays as it will latch on at boot and your wifi will not connect until you remove the GPIO connection from the relay.
  D8 Will also latch on and cause conflict do not use this. - See more at: http://www.esp8266.com/viewtopic.php?f=40&t=6732#sthash.IzOPRBbl.dpuf





*/

// Import required libraries
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <aREST.h>
#include <Servo.h> 


// Variables to be exposed to the API
int build = 29;


// Create aREST instance
aREST rest = aREST();


// PRO Clients
WiFiClient espClient;
PubSubClient client(espClient);

// PRO Create aREST instance
aREST restPro = aREST(client);

// aREST Pro key (that you can get at dashboard.arest.io)
char * key = "your_pro_key";

char * deviceName = "AppCar";


// WiFi parameters
const char* ssidAP = "AppCar";
const char* passwordAP = "appshedrocks";
const char* ssidInternet = "AppShed";
const char* passwordInternet = "appshedrocks";

// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);


// Pro Functions
void callback(char* topic, byte* payload, unsigned int length);


// Declare functions to be exposed to the API
int calibrate(String command);
int commands(String command);
int logo(String command);
int runCommands(String command);
int attachServos(String command);


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

  
int pinMLF = 2; // default pin for left motor Forward
int pinMLB = 3; // left motor backwards
int pinMRF = 6; // right motor Forward
int pinMRB = 7; // right motor Backwards
//int pinLPWM = 4; // default pin for left motor PWM
//int pinRPWM = 5; // default pin for right motor PWM
int pinPen = 4; // default pin for Logo Pen
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
int gpio[] = {16,5,4,0,2,14,12,13,15};
int logoQueue[20][2]; // [command#] [command,value]
int servoArray[4]; // the pin numbers for the servos


void setup(void)
{
  // Start Serial
  Serial.begin(115200);

  // Keep the PWM Range the same as the Uno
  // DEPRECATED analogWriteRange(255);

  // Pin 4 is the default LED OUTPUT
  pinMode(gpio[4], OUTPUT);
  digitalWrite(gpio[4],0); // 0 turns the blue LED on


  // Allocate motor pins as OUTPUT and turn off
  pinMode(gpio[pinMLF], OUTPUT);
  pinMode(gpio[pinMLB], OUTPUT);
  pinMode(gpio[pinMRF], OUTPUT);
  pinMode(gpio[pinMRB], OUTPUT);  
//  pinMode(gpio[pinLPWM], OUTPUT);
//  pinMode(gpio[pinRPWM], OUTPUT);

  digitalWrite(gpio[pinMLF],0);
  digitalWrite(gpio[pinMLB],0);
  digitalWrite(gpio[pinMRF],0);
  digitalWrite(gpio[pinMRB],0);
//  analogWrite(gpio[pinLPWM],0);
//  analogWrite(gpio[pinRPWM],0);


  // see if D0 and D1 are bridged
  pinMode(gpio[0], OUTPUT);
  pinMode(gpio[1], INPUT);
  // first test, turn ON D0;
  digitalWrite(gpio[0],1);
  if(digitalRead(gpio[1]) == 1){
    // second test, turn off D0
    digitalWrite(gpio[0],0);
    if(digitalRead(gpio[1]) == 0){
      bridged = 1;
    }
  }
  Serial.println("");
  Serial.print("Bridge ");
  Serial.println(bridged);


  // BRIDGED: Use Pro Cloud Service
  if(bridged){

        Serial.println("");
        Serial.println("-----------------------------");
        Serial.println("Mode: Pro");
        Serial.println("-----------------------------");

        // Set aREST key
        restPro.setKey(key, client);
      
        // Set callback
        client.setCallback(callback);
      
        // Init variables and expose them to REST API
        restPro.variable("build",&build);
      
        // Give name to device
        restPro.set_name(deviceName);
      
      
        // Function to be exposed
        restPro.function("calibrate",calibrate);
        restPro.function("commands",commands);
        restPro.function("logo",logo);
        restPro.function("runCommands",runCommands);
        restPro.function("attachServos",attachServos);
      
        // Connect to WiFi
        WiFi.begin(ssidInternet, passwordInternet);
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
        Serial.print("SSID: ");
        Serial.println(ssidInternet);
        Serial.print("Password: ");
        Serial.println(passwordInternet);
        Serial.println("");
      
        // Set output topic
        char* out_topic = restPro.get_topic();
    
  } else {
  // NOT BRIDGED: Use Local AP

        Serial.println("");
        Serial.println("-----------------------------");
        Serial.println("Mode: Local AP");
        Serial.println("-----------------------------");

        // Init variables and expose them to REST API
        rest.variable("build",&build);
        
      
        // Function to be exposed
        rest.function("calibrate",calibrate);
        rest.function("commands",commands);
        rest.function("logo",logo);
        rest.function("runCommands",runCommands);
        rest.function("attachServos",attachServos);
      
        // Give name & ID to the device (ID should be 6 characters long)
        rest.set_id("local");
        rest.set_name(deviceName);
      
      
        
        // Setup WiFi network
        WiFi.softAP(ssidAP, passwordAP);
        Serial.println("");
        Serial.println("WiFi created");
        Serial.print("SSID: ");
        Serial.println(ssidAP);
        Serial.print("Password: ");
        Serial.println(passwordAP);
        Serial.println("");
      
        // Start the server
        server.begin();
        Serial.println("Server started");
      
        // Print the IP address
        IPAddress myIP = WiFi.softAPIP();
        Serial.print("AP IP address: ");
        Serial.println(myIP);

     
  }
  

}

void loop() {


  runLogo();

  runCommands("");


  // BRIDGED: Use Pro Cloud Service
  if(bridged){
      // Connect to the cloud
      restPro.handle(client);
    
  }else {
      // Handle REST calls
      WiFiClient client = server.available();
      if (!client) {
        return;
      }
      while(!client.available()){
        runCommands("");
        delay(1);
      }
      rest.handle(client);    
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
  return found>index ? data.substring(strIndex[0], strIndex[1]) : "";
}







// Custom function accessible by the API
int commands(String command) {

  // Commands will be sent in this format:
  // format,pin,value,duration:format,pin,value,duration:....
  // format is 0-analog or 1-digital


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
    valueInt = data.toInt();
    
    data = getValue(command, ',', item_counter++);
    // default duration is 0 (if ""), i.e. indefinite duration
    durationInt = data.toInt();

    int commandIndex = commandQueueIndex[pinInt];


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

Serial.print("executeLogo() ");
Serial.print(command);
Serial.print(",");
Serial.println(value);

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
Serial.print("currentLogoCommandExpiry ");
Serial.println(currentLogoCommandExpiry);

  // Make sure Motor Pins have been configured
  if(motorPinsConfigured == 0){
    initialiseMotorPins();
    pinMode(gpio[pinPen], OUTPUT);
    digitalWrite(gpio[pinPen],1); // Put the pen down by default.

  }


  // Use pinMLF for setting direction: 1/0
  // Use pinMLB for speed, analog
  
  // set the required pins
  if(command == 0){
    digitalWrite(gpio[pinMLF],0);
    analogWrite(gpio[pinMLB],0);
    digitalWrite(gpio[pinMRF],0);
    analogWrite(gpio[pinMRB],0);
  }
  if(command == 1){
    digitalWrite(gpio[pinMLF],1);
    analogWrite(gpio[pinMLB],(maxPWM - leftPWMF));
    digitalWrite(gpio[pinMRF],1);
    analogWrite(gpio[pinMRB],(maxPWM - rightPWMF));
  }
  if(command == 2){
//    analogWrite(gpio[pinLPWM],leftPWMB);
//    analogWrite(gpio[pinRPWM],rightPWMB);
    
    digitalWrite(gpio[pinMLF],0);
    analogWrite(gpio[pinMLB],leftPWMB);
    digitalWrite(gpio[pinMRF],0);
    analogWrite(gpio[pinMRB],rightPWMB);
  }
  if(command == 3){
    digitalWrite(gpio[pinMLF],0);
    analogWrite(gpio[pinMLB],leftPWMB);
    digitalWrite(gpio[pinMRF],1);
    analogWrite(gpio[pinMRB],(maxPWM - rightPWMF));
  }
  if(command == 4){
    digitalWrite(gpio[pinMLF],1);
    analogWrite(gpio[pinMLB],(maxPWM - leftPWMF));
    digitalWrite(gpio[pinMRF],0);
    analogWrite(gpio[pinMRB],rightPWMB);
  }
  if(command == 5){
    digitalWrite(gpio[pinMLF],0);
    analogWrite(gpio[pinMLB],0);
    digitalWrite(gpio[pinMRF],1);
    analogWrite(gpio[pinMRB],(maxPWM - rightPWMF));
  }
  if(command == 6){
    digitalWrite(gpio[pinMLF],1);
    analogWrite(gpio[pinMLB],(maxPWM - leftPWMF));
    digitalWrite(gpio[pinMRF],0);
    analogWrite(gpio[pinMRB],0);
  }
  if(command == 7){
//    analogWrite(gpio[pinLPWM],leftPWMB);
//    analogWrite(gpio[pinRPWM],rightPWMB);

    digitalWrite(gpio[pinMLF],0);
    analogWrite(gpio[pinMLB],0);
    digitalWrite(gpio[pinMRF],0);
    analogWrite(gpio[pinMRB],rightPWMB);
  }
  if(command == 8){
//    analogWrite(gpio[pinLPWM],leftPWMB);
//    analogWrite(gpio[pinRPWM],rightPWMB);

    digitalWrite(gpio[pinMLF],0);
    analogWrite(gpio[pinMLB],leftPWMB);
    digitalWrite(gpio[pinMRF],0);
    analogWrite(gpio[pinMRB],0);
  }
  if(command == 9){
    // Pen Up. Default behaviour turns the pinPen on and off
    digitalWrite(gpio[pinPen],0);    
  }
  if(command == 10){
    // Pen Down. Default behaviour turns the pinPen on and off. Other Option to move servo
    digitalWrite(gpio[pinPen],1);
  }


  /*          
   *   OLD WAY - Using digitalWrite.
   *   REPLACED WITH analogWrite
          //  analogWrite(gpio[pinLPWM],leftPWMF);
          //  analogWrite(gpio[pinRPWM],rightPWMF);
          
            // set the required pins
            if(command == 0){
              digitalWrite(gpio[pinMLF],0);
              digitalWrite(gpio[pinMLB],0);
              digitalWrite(gpio[pinMRF],0);
              digitalWrite(gpio[pinMRB],0);
            }
            if(command == 1){
              digitalWrite(gpio[pinMLF],1);
              digitalWrite(gpio[pinMLB],0);
              digitalWrite(gpio[pinMRF],1);
              digitalWrite(gpio[pinMRB],0);
            }
            if(command == 2){
          //    analogWrite(gpio[pinLPWM],leftPWMB);
          //    analogWrite(gpio[pinRPWM],rightPWMB);
              
              digitalWrite(gpio[pinMLF],0);
              digitalWrite(gpio[pinMLB],1);
              digitalWrite(gpio[pinMRF],0);
              digitalWrite(gpio[pinMRB],1);
            }
            if(command == 3){
              digitalWrite(gpio[pinMLF],0);
              digitalWrite(gpio[pinMLB],1);
              digitalWrite(gpio[pinMRF],1);
              digitalWrite(gpio[pinMRB],0);
            }
            if(command == 4){
              digitalWrite(gpio[pinMLF],1);
              digitalWrite(gpio[pinMLB],0);
              digitalWrite(gpio[pinMRF],0);
              digitalWrite(gpio[pinMRB],1);
            }
            if(command == 5){
              digitalWrite(gpio[pinMLF],0);
              digitalWrite(gpio[pinMLB],0);
              digitalWrite(gpio[pinMRF],1);
              digitalWrite(gpio[pinMRB],0);
            }
            if(command == 6){
              digitalWrite(gpio[pinMLF],1);
              digitalWrite(gpio[pinMLB],0);
              digitalWrite(gpio[pinMRF],0);
              digitalWrite(gpio[pinMRB],0);
            }
            if(command == 7){
          //    analogWrite(gpio[pinLPWM],leftPWMB);
          //    analogWrite(gpio[pinRPWM],rightPWMB);
          
              digitalWrite(gpio[pinMLF],0);
              digitalWrite(gpio[pinMLB],0);
              digitalWrite(gpio[pinMRF],0);
              digitalWrite(gpio[pinMRB],1);
            }
            if(command == 8){
          //    analogWrite(gpio[pinLPWM],leftPWMB);
          //    analogWrite(gpio[pinRPWM],rightPWMB);
          
              digitalWrite(gpio[pinMLF],0);
              digitalWrite(gpio[pinMLB],1);
              digitalWrite(gpio[pinMRF],0);
              digitalWrite(gpio[pinMRB],0);
            }
            if(command == 9){
              // Pen Up. Default behaviour turns the pinPen on and off
              digitalWrite(gpio[pinPen],0);    
            }
            if(command == 10){
              // Pen Down. Default behaviour turns the pinPen on and off. Other Option to move servo
              digitalWrite(gpio[pinPen],1);
            }
  */
            
  return 1;
}



void initialiseMotorPins(){
  // Configure the motor pins as outputs
  
  // turn pin 4 off
  pinMode(gpio[pinMLF], OUTPUT);
  pinMode(gpio[pinMLB], OUTPUT);
  pinMode(gpio[pinMRF], OUTPUT);
  pinMode(gpio[pinMRB], OUTPUT);
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
  // Update the constant c setting it to value

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

  // format: 0: analog (pwm), 1: digital

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
          
        pinMode(gpio[pinInt], OUTPUT);
        if(format == 0){
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
          digitalWrite(gpio[pinInt],value);
        }
  Serial.print("runCommand... format:");
  Serial.print(format);
  Serial.print(" pin:");
  Serial.print(pinInt);
  Serial.print(" value:");
  Serial.println(value);
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



// Custom function accessible by the API
int logo(String command) {
Serial.print("logo() ");
Serial.println(command);

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
Serial.print("addLogoCommand() ");
Serial.print(command);
Serial.print(",");
Serial.println(value);

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


