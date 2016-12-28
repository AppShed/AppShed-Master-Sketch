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
*/

// Import required libraries
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <aREST.h>
#include <Servo.h> 

// Create aREST instance
aREST rest = aREST();

// PRO Clients
WiFiClient espClient;
PubSubClient client(espClient);

// PRO Create aREST instance
aREST restPro = aREST(client);

// aREST Pro key (that you can get at dashboard.arest.io)
char * key = "your_pro_key";


// WiFi parameters
const char* ssidAP = "AppShed-GK";
const char* ssid = "AppShed";
const char* password = "appshedrocks";

// The port to listen for incoming TCP connections
#define LISTEN_PORT           80

// Create an instance of the server
WiFiServer server(LISTEN_PORT);

// Variables to be exposed to the API
int build = 24;

// Pro Functions
void callback(char* topic, byte* payload, unsigned int length);


// Declare functions to be exposed to the API
int commands(String command);
int drive(String command);
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

int leftF = 4; // default pin for left motor Forward
int leftB = 5; // left motor backwards
int rightF = 6; // right motor Forward
int rightB = 7; // right motor Backwards


int commandQueue[9][19][3]; // [pin 0-19][command# 0-max][0:format 1:value 2:duration]
int commandQueueIndex[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // the current index for each pin in commandQueue
unsigned long commandTimeout[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}; // the time at which the command for a pin times out
int gpio[] = {16,5,4,0,2,14,12,13,15};
int servoArray[4]; // the pin numbers for the servos


void setup(void)
{
  // Start Serial
  Serial.begin(115200);

  // turn pin 4 off
  pinMode(gpio[4], OUTPUT);
  digitalWrite(gpio[4],0);



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
        restPro.set_name("appcarbt");
      
      
        // Function to be exposed
        restPro.function("commands",commands);
        restPro.function("runCommands",runCommands);
        restPro.function("attachServos",attachServos);
      
        // Connect to WiFi
        WiFi.begin(ssid, password);
        while (WiFi.status() != WL_CONNECTED) {
          delay(500);
          Serial.print(".");
        }
        Serial.println("");
        Serial.println("WiFi connected");
      
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
        rest.function("commands",commands);
        rest.function("drive",drive);
        rest.function("runCommands",runCommands);
        rest.function("attachServos",attachServos);
      
        // Give name & ID to the device (ID should be 6 characters long)
        rest.set_id("local");
        rest.set_name("appcargk");
      
      
        
        // Setup WiFi network
        WiFi.softAP(ssidAP, password);
        Serial.println("");
        Serial.println("WiFi created");
      
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
int drive(String command) {
Serial.print("drive() ");
Serial.println(command);

  // Drives two motors in various directions.
  // Default pins for left and right motors are set by variables leftF, leftB, rightF, rightB
  // commands will be sent in this format:
  // direction[,duration,speed]: <repeat> : ...q
  // Direction is 
  //    0 (stop)
  //    1 (forward)
  //    2 (backward)
  //    3 (forward left)
  //    4 (forward right)
  //    5 (backward left)
  //    6 (backward right)
  //    7 (rotate left/anti-clockwise)
  //    8 (rotate right/clockwise)
  // Duration (optional) is time in milliseconds. Defaults to 0 (continous until next command).
  // Speed (optional) is percentage speed of driving, 0-100. Defaults to 100.

  String str_Command;
  int item_counter = 0;
  while(1){
    str_Command = getValue(command, ':', item_counter++);
    if(str_Command == "")
      break;
      
    setDriveCommand(str_Command);

  }
  return item_counter;
}

void setDriveCommand(String command){
Serial.print("setDriveCommand() ");
Serial.println(command);

  // expecting command direction[,duration,speed]

  String data;
  String directionStr;
  String durationStr;
  String speedStr;


  
  int item_counter = 0;

  while(1){
    // direction
    directionStr = getValue(command, ',', item_counter++);
    if(directionStr == "")
      break;

    // duration
    durationStr = getValue(command, ',', item_counter++);
    if(durationStr == "")
      durationStr = "0";

    // speed
    speedStr = getValue(command, ',', item_counter++);
    if(speedStr == "")
      speedStr = "100";

Serial.println("setDriveCommand... " + directionStr + "," + durationStr + "," + speedStr);

    // 9 different directions
    // expecting command format,pin,value,duration
    if(directionStr == "0"){
      addCommandToQueue("1,4,0," + durationStr);       
      addCommandToQueue("1,5,0," + durationStr);       
      addCommandToQueue("1,6,0," + durationStr);       
      addCommandToQueue("1,7,0," + durationStr);       
    }
    if(directionStr == "1"){
      addCommandToQueue("1,4,1," + durationStr);       
      addCommandToQueue("1,5,0," + durationStr);       
      addCommandToQueue("1,6,1," + durationStr);       
      addCommandToQueue("1,7,0," + durationStr);       
    }
    if(directionStr == "2"){
      addCommandToQueue("1,4,0," + durationStr);       
      addCommandToQueue("1,5,1," + durationStr);       
      addCommandToQueue("1,6,0," + durationStr);       
      addCommandToQueue("1,7,1," + durationStr);       
    }
    if(directionStr == "3"){
      addCommandToQueue("1,4,0," + durationStr);       
      addCommandToQueue("1,5,0," + durationStr);       
      addCommandToQueue("1,6,1," + durationStr);       
      addCommandToQueue("1,7,0," + durationStr);       
    }
    if(directionStr == "4"){
      addCommandToQueue("1,4,1," + durationStr);       
      addCommandToQueue("1,5,0," + durationStr);       
      addCommandToQueue("1,6,0," + durationStr);       
      addCommandToQueue("1,7,0," + durationStr);       
    }
    if(directionStr == "5"){
      addCommandToQueue("1,4,0," + durationStr);       
      addCommandToQueue("1,5,1," + durationStr);       
      addCommandToQueue("1,6,0," + durationStr);       
      addCommandToQueue("1,7,0," + durationStr);       
    }
    if(directionStr == "6"){
      addCommandToQueue("1,4,0," + durationStr);       
      addCommandToQueue("1,5,0," + durationStr);       
      addCommandToQueue("1,6,0," + durationStr);       
      addCommandToQueue("1,7,1," + durationStr);       
    }
    if(directionStr == "7"){
      addCommandToQueue("1,4,0," + durationStr);       
      addCommandToQueue("1,5,1," + durationStr);       
      addCommandToQueue("1,6,1," + durationStr);       
      addCommandToQueue("1,7,0," + durationStr);       
    }
    if(directionStr == "8"){
      addCommandToQueue("1,4,1," + durationStr);       
      addCommandToQueue("1,5,0," + durationStr);       
      addCommandToQueue("1,6,0," + durationStr);       
      addCommandToQueue("1,7,1," + durationStr);       
    }
  }

}

