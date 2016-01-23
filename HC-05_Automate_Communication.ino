/*
 * Project:         HC-05 Automate Bonding with Bluetooth device
 * Description:     This assisted bonding is specifically geared towards bonding with a GPS Bluetooth device
 *                  Was written to assist automation of the process in the SmartClock-GPS Project
 *                  (Could possibly be further developed to become an abstracted library for controlling HC-05 bluetooth modules...)
 * Author:          John Romano D'Orazio
 * Author Website:  http://www.johnromanodorazio.com
 * Author Email:    john.dorazio@cappellaniauniroma3.org
 * License:         GPLv3 (see the full license at bottom of this file)
 * Board:           Atmega 1284P on a breadboard using bootloader "maniacbug Mighty 1284P 16MHZ using Optiboot"
 * Bootloader:      https://github.com/JChristensen/mighty-1284p/tree/v1.6.3 (for usage with Arduino 1.6.3 and higher)
 * Last Modified:   23 January 2016
 * 
 * A project of the Microcontrollers Users Group at Roma Tre University
 * MUG Roma3 http://muglab.uniroma3.it
 * 
 * Preliminary Requirements:
 * HC-05 DEVICE MUST BE PREVIOUSLY SET AS MASTER, OTHERWISE INQUIRY WILL NOT WORK! (AT+ROLE=1)
 * IS BEST TO MANUALLY SET INQUIRY SCAN PARAMETERS (AT+IPSCAN) TO A GOOD DEFAULT (1024,512,1024,512)
 * ALSO INQUIRY MODE TO A GOOD DEFAULT (AT+INQM=1,15,12) RSSI MODE,MAX 5 DEVICES,TIMEOUT (MULTIPLY TIMEOUT BY 1.28 TO GET ACTUAL NUMBER OF SECONDS: 12*1.28=15.36)
 * PREPARE PSWD SAME AS DEVICE TO CONNECT TO (in case of GPS Bluetooth device is "0000", so AT+PSWD=0000)
 * -> LATEST VERSION OF THIS SCRIPT AUTOMATES THE ABOVE REQUIRED SETTINGS EVERY TIME THE HC-05 MODULE IS SET TO AT-MODE, TO AVOID THEM BEING ACCIDENTALLY LEFT OUT
 * 
 * Useful literature:
 * -> http://wiki.iteadstudio.com/Serial_Port_Bluetooth_Module_(Master/Slave)_:_HC-05
 * -> http://www.hobbytronics.co.uk/datasheets/EGBT-bluetooth-module.pdf (different module but same kind of functionality)
 * 
 * LOGICAL STEPS IMPLEMENTED BY THE PROGRAM:
 *  1) START DEVICE IN COMMUNICATION MODE
 *  2) CHECK DEVICE STATE (CONNECTED / DISCONNECTED) USING STATE PIN
 *      a-> IF CONNECTED, THEN LISTEN FOR GPS NMEA DATA OVER SERIAL
 *      b-> ELSE ENTER AT MODE (continue 3) )
 *  3) IF AT MODE, COUNT RECENT CONNECTED DEVICES (AT+ADCN)
 *      a-> IF COUNT > 0 THEN CHECK MOST RECENTLY USED DEVICE (AT+MRAD), THEN TRY TO LINK TO IT (AT+LINK=<device address>)
 *         i-> IF OK THEN WE ARE IN COMMUNICATION MODE, CHECK FOR CONNECTED STATE, IF YES THEN LISTEN FOR GPS NMEA DATA OVER SERIAL
 *        ii-> IF LINK FAILS, THEN THE GPS DEVICE IS PROBABLY TURNED OFF OR OUT OF RANGE, START SCANNING FOR NEARBY DEVICES (goto b->)
 *      b-> ELSE START SCANNING NEARBY DEVICES, COLLECT THEIR ADDRESSES IN AN ARRAY, THEN CYCLE THROUGH THEM AND PROMPT USER TO CONFIRM LINK (TRANSLATE DEVICE ADDRESS TO DEVICE NAME TO ASSIST USER)
 *         i-> IF USER CONFIRMS, THEN BIND TO THIS ADDRESS (AT+BIND=<device address>), SET CONNECTION MODE TO ONLY BOUND DEVICE (AT+CMODE=0) AND TRY TO LINK (AT+LINK=<device address>)
 *            i/a-> IF OK THEN WE ARE IN COMMUNICATION MODE, CHECK FOR CONNECTED STATE, IF YES THEN LISTEN FOR GPS NMEA DATA OVER SERIAL
 *            i/b-> IF LINK FAILS, THEN THE GPS DEVICE IS PROBABLY TURNED OFF OR OUT OF RANGE, START SCANNING FOR NEARBY DEVICES (goto b->)         
 *        ii-> ELSE CYCLE TO NEXT DEVICE AND PROMPT USER AGAIN
 *       iii-> IF CYCLE ENDS WITHOUT A SINGLE USER CONFIRMATION, SCAN DEVICES AGAIN (goto b->)
 *      
 * This sketch was written for use with an Atmega 1284P on a breadboard, using the "maniacbug Mighty 1284P 16MHZ using optiboot" bootloader
 * The Atmega 1284P has more then one hardware UART, allowing us to use both Serial and Serial1, thus avoiding SoftwareSerial
 * (in my personal experience, perhaps depending also on the baud rate, SoftwareSerial can result in corrupted or spurious serial data, so I prefer hardware serial)
 * 
 * For wire connections between the HC-05 bluetooth module and the 1284P, see https://github.com/mugroma3/HC05_Manual_AT_Mode/blob/master/HC-05_Manual_AT_Mode.ino
 * 
 * For this script, the Arduino Serial Monitor must be set to 38400 baud and send only "LF" (or "NL" newline \n)
 *      
 *      
 *      TODO: consider using PROGRAMSTATE definitions instead of booleans as PROGRAM STATE FLAGS...
 *            then usage would be like "IF(PROGRAMSTATE == SCANNINGDEVICES){}"
 *            and we could use a SWITCH CASE instead of IF...ELSE IF
 */

#include "Timer.h"                     //http://github.com/JChristensen/Timer

/* DEFINE OUR CONSTANTS */

#define LED             0 // led digital pin

#define HC05_STATE_PIN  12 //hc-05 state digital pin
#define HC05_KEY_PIN    13 //hc-05 key digital pin
#define HC05_EN_PIN     14 //hc-05 enable digital pin

#define AT_MODE               HIGH
#define COMMUNICATION_MODE    LOW

//module ROLE definitions
#define SLAVE         0
#define MASTER        1
#define SLAVE_LOOP    2

//module STATE definitions
#define DISCONNECTED  0
#define CONNECTED     1

//module max inquire devices
#define MAX_DEVICES   15

//module CMODE options
#define CONNECT_BOUND       0
#define CONNECT_ANY         1
#define CONNECT_SLAVE_LOOP  2

const String ERRORMESSAGE[29] = {
  "Command Error/Invalid Command",
  "Results in default value",
  "PSKEY write error",
  "Device name is too long (>32 characters)",
  "No device name specified (0 length)",
  "Bluetooth address NAP is too long",
  "Bluetooth address UAP is too long",
  "Bluetooth address LAP is too long",
  "PIO map not specified (0 length)",
  "Invalid PIO port Number entered",
  "Device Class not specified (0 length)",
  "Device Class too long",
  "Inquire Access Code not Specified (0 length)",
  "Inquire Access Code too long",
  "Invalid Inquire Access Code entered",
  "Pairing Password not specified (0 length)",
  "Pairing Password too long (> 16 characters)",
  "Invalid Role entered",
  "Invalid Baud Rate entered",
  "Invalid Stop Bit entered",
  "Invalid Parity Bit entered",
  "No device in the Pairing List",
  "SPP not initialized",
  "SPP already initialized",
  "Invalid Inquiry Mode",
  "Inquiry Timeout occured",
  "Invalid/zero length address entered",
  "Invalid Security Mode entered",
  "Invalid Encryption Mode entered"
};

//possible states for the current program
enum{
  INITIALSTATECHECK,
  DO_ADCN,
  COUNTINGRECENTDEVICES,
  COUNTEDRECENTDEVICES,
  SEARCHAUTHENTICATEDDEVICE,
  CONNECTINGRECENTDEVICE,
  SETTINGCONNECTIONMODE,
  INITIATINGINQUIRY,
  INQUIRINGDEVICES,
  CONFRONTINGUSER,
  SETTINGBINDADDRESS,
  CONNECTINGTODEVICE,
  LISTENNMEA
}PROGRAMSTATE;

int OLDPROGRAMSTATE;
boolean PROGRAMSTATECHANGED;

/* INITIALIZE OUR GLOBAL VARIABLES */

//instantiate the timer object
Timer t;
int dynamicEvent;

//these booleans represent program state
//perhaps they could be made into constants, with a single variable representing program state...
//I won't do this just yet, because as things stand combinations of these states are sometimes utilized...
//(specifically the first three; all others seem to be mutually exclusive...)
boolean SETTINGHC05MODE = false;
//boolean DO_ADCN = false;
boolean INITIALIZING = false;

int HC05_MODE;                  //can be AT_MODE or COMMUNICATION_MODE
int HC05_STATE;                 //can be CONNECTED or DISCONNECTED
int HC05_OLDSTATE;              //can be CONNECTED or DISCONNECTED
int deviceCount = 0;
int recentDeviceCount = 0;
int currentDeviceIdx = 0;
int currentCMODE;
int currentFunctionStep = 0;

String incoming;
String outgoing;
String devices[MAX_DEVICES];
String currentDeviceAddr;
String currentDeviceName;


// the setup function runs once when you press reset or power the board
void setup() {
  pinMode(LED, OUTPUT);
  pinMode(HC05_STATE_PIN, INPUT);
  pinMode(HC05_KEY_PIN, OUTPUT);
  pinMode(HC05_EN_PIN, OUTPUT);

  Serial.begin(38400);
  while(!Serial){} //wait until Serial is ready
  Serial.println("Serial communication is ready!");
  
  Serial1.begin(38400);
  while(!Serial1){} //wait until Serial1 is ready
  Serial.println("HC-05 serial is ready too!");

  PROGRAMSTATE = INITIALSTATECHECK;
  
  //Start the HC-05 module in communication mode
  HC05_MODE = COMMUNICATION_MODE;  
  //Set_HC05_MODE function uses HC05_MODE global instead of sending the mode in as a parameter,
  //because this function is called in multiple steps by the timer object, as a callback,
  //making it complicated to handle parameters...
  Set_HC05_MODE(); 
        
  //time = millis();
}

// the loop function runs over and over again forever
void loop() {
  t.update();
  if(PROGRAMSTATE != OLDPROGRAMSTATE){
    PROGRAMSTATECHANGED = true;
    OLDPROGRAMSTATE = PROGRAMSTATE;
  }
  else{
    PROGRAMSTATECHANGED = false;
  }
  
  if(SETTINGHC05MODE == false && PROGRAMSTATE == INITIALSTATECHECK){
    HC05_STATE = Check_HC05_STATE();
    if(HC05_STATE == CONNECTED){
      Serial.println("HC-05 is connected and is now listening for NMEA data...");
      PROGRAMSTATE = LISTENNMEA;
    }
    else if(HC05_STATE == DISCONNECTED){
      Serial.println("HC-05 is not connected, so let's see what we can do about that.");
      HC05_MODE = AT_MODE;      
      Set_HC05_MODE();
    }
  }
  else if(SETTINGHC05MODE == false && PROGRAMSTATE == DO_ADCN){
    CountRecentAuthenticatedDevices();
  }
  
  if(PROGRAMSTATE == LISTENNMEA){
    //serial listen for nmea data and all that stuff
  }


 // read serial data from serial monitor and send to HC-05 module:
  if (Serial.available() > 0) {
    outgoing = Serial.readStringUntil('\n');
    Serial.println(outgoing);
    if(PROGRAMSTATE == CONFRONTINGUSER){
      if(outgoing.startsWith("Y")){
        currentCMODE = SetConnectionMode(CONNECT_BOUND);
      }
      else if(outgoing.startsWith("N")){
        currentDeviceIdx++;
        if(currentDeviceIdx < deviceCount){
          while(devices[currentDeviceIdx] == devices[currentDeviceIdx-1]){
            if(currentDeviceIdx < deviceCount-1){ currentDeviceIdx++; }
            else{ break; }            
          }
          if(devices[currentDeviceIdx] != devices[currentDeviceIdx-1]){
            currentDeviceAddr = devices[currentDeviceIdx];
            ConfrontUserWithDevice(currentDeviceAddr);
          }
          else{
            Serial.println("Devices are exhausted. Perhaps you need to turn on the correct Bluetooth Device and make it searchable.");            
            InitiateInquiry();
          }
        }
        else{
          Serial.println("Devices are exhausted. Perhaps you need to turn on the correct Bluetooth Device and make it searchable.");
          InitiateInquiry();
        }
      }
    }
    else{
      Serial1.println(outgoing);
    }
    //Serial1.write(Serial.read()); 
  }
  
  // read serial data from HC-05 module and send to serial monitor:
  if(Serial1.available() > 0) {
    if(SETTINGHC05MODE && INITIALIZING==false){
      Serial1.read(); //just throw it away, don't do anything with it
    }
    else if(INITIALIZING){
      //let's not worry about parsing all the results... just print them out
      Serial.write(Serial1.read());
    }
    else{
      incoming = "";
  
      byte inc = Serial1.read(); //check the incoming character    
      if(inc != 0 && inc < 127){ //if the incoming character is not junk then continue reading the string
        
        incoming = char(inc) + Serial1.readStringUntil('\n');
        Serial.println(incoming);

        if(PROGRAMSTATECHANGED){
          
          switch(PROGRAMSTATE){
            
            case COUNTINGRECENTDEVICES:
              if(incoming.startsWith("+ADCN")){
                Serial.println("Response from recent device count is positive!");
                String deviceCountStr = incoming.substring(incoming.indexOf(':')+1); //should ':' be ' '?
                recentDeviceCount = deviceCountStr.toInt();
                
                flushOkString();
                
                if(recentDeviceCount>0){
                  String dev = (recentDeviceCount==1?" device":" devices");
                  String mex = "HC-05 has been recently paired with " + deviceCountStr + dev;
                  Serial.println(mex);
                  CheckMostRecentAuthenticatedDevice();
                }else{
                  Serial.println("There do not seem to be any recent devices.");
                  //We will have to initiate inquiry of nearby devices. First, we have to make sure we that the HC-05 is set to connect to any device.
                  currentCMODE = SetConnectionMode(CONNECT_ANY);
                }
              }
              else{
                Serial.println("COUNTINGRECENTDEVICES ???");
              }

              break;
              
            case COUNTEDRECENTDEVICES:
              if(incoming.startsWith("+MRAD")){
                Serial.println("I believe I have detected the Address of the most recent device...");
                
                flushOkString();
      
                int idx = incoming.indexOf(':'); // should this be a space perhaps...
                //int idx1 = incoming.indexOf(',');
                if(idx != -1){ // && idx1 != -1
                  String addr = incoming.substring(idx+1); //,idx
                  addr.replace(':',',');
                  addr.trim();
                  currentDeviceAddr = addr;
                  SearchAuthenticatedDevice(currentDeviceAddr); 
                }
                else{
                  Serial.println("The response string surprised me, I don't know what to say.");
                }
              }
              else{
                Serial.println("The response string does not seem to be an address for the most recent device...");
              }
              break;

            case SEARCHAUTHENTICATEDDEVICE:
              if(incoming.startsWith("OK")){
                  Serial.println("Now ready to connect to device with address <" + currentDeviceAddr + ">.");
                  ConnectRecentAuthenticatedDevice(currentDeviceAddr);
              }
              else if(incoming.startsWith("FAIL")){
                Serial.println("Unable to prepare for connection with device whose address is <" + currentDeviceAddr + ">.");
              }
              else{
                Serial.println("SEARCHAUTHENTICATEDDEVICE ???");
              }
              break;
              
            case CONNECTINGRECENTDEVICE:
              if(incoming.startsWith("OK")){
                Serial.println("HC-05 is now connected to most recent authenticated device");
              }
              else if(incoming.startsWith("FAIL")){
                Serial.println("Connection to most recent authenticated device has failed.");
                InquireDevices();
              }
              else{
                Serial.println("CONNECTINGRECENTDEVICE ???");
              }
              break;

            case SETTINGCONNECTIONMODE:
              if(incoming.startsWith("OK")){
                if(currentCMODE == CONNECT_ANY){
                  Serial.println("CMODE successfully set to connect to any device.");
                  InitiateInquiry();
                }
                else if(currentCMODE == CONNECT_BOUND){
                  Serial.println("CMODE successfully set to connect only to bound device.");
                  SetBindAddress();
                }
              }
              break;

            case INITIATINGINQUIRY:
              if(incoming.startsWith("OK")){
                InquireDevices();
              }
              else if(incoming.startsWith("ERROR")){
                int idx1 = incoming.indexOf('(');
                int idx2 = incoming.indexOf(')');            
                String errCode = incoming.substring(idx1+1,idx2);            
                if(errCode == "17"){ //already initiated, no problem, continue just the same
                  InquireDevices();  
                }
                else{
                  Serial.println(incoming + ": " + getErrorMessage(errCode));
                }                
              }
              break;
              
            case INQUIRINGDEVICES:
              if(incoming.startsWith("OK")){
                Serial.println("Finished inquiring devices.");
                currentDeviceIdx=0;
                if(deviceCount > 0){
                  currentDeviceAddr = devices[currentDeviceIdx];
                  ConfrontUserWithDevice(currentDeviceAddr);  
                }        
                else{
                  Serial.print("Sorry, I have not found any useful devices. Trying again.");
                  InitiateInquiry();
                }
              }
              else if(incoming.startsWith("+INQ")){
                int idx = incoming.indexOf(':');
                int idx2 = incoming.indexOf(',');
                String addr = incoming.substring(idx+1,idx2);
                addr.replace(':',',');
                addr.trim();
                devices[deviceCount] = addr;
                deviceCount++;
              }
              break;

            case CONFRONTINGUSER:
              if(incoming.startsWith("+RNAME")){                    
                currentDeviceName = incoming.substring(incoming.indexOf(':')+1);
                
                flushOkString();
                
                Serial.println("Would you like to connect to '" + currentDeviceName + "'? Please type Y or N.");
              }
              else if(incoming.startsWith("FAIL")){
                Serial.println("Could not retrieve name of detected device... continuing to next");
                currentDeviceIdx++;
                if(currentDeviceIdx < deviceCount){
                  currentDeviceAddr = devices[currentDeviceIdx];
                  ConfrontUserWithDevice(currentDeviceAddr);
                }
                else{
                  Serial.println("Devices are exhausted. Searching again.");
                  InitiateInquiry();
                }
              }
              else{
                Serial.println("There was an error retrieving the device name... continuing to next");
                currentDeviceIdx++;
                if(currentDeviceIdx < deviceCount){
                  currentDeviceAddr = devices[currentDeviceIdx];
                  ConfrontUserWithDevice(currentDeviceAddr);
                }
                else{
                  Serial.println("Devices are exhausted. Searching again.");
                  InitiateInquiry();
                }
              }
              break;

            case SETTINGBINDADDRESS:
              if(incoming.startsWith("OK")){
                Serial.println("Bound HC-05 to bluetooth device.");
                LinkToCurrentDevice(currentDeviceAddr);          
              }
              else{
                Serial.println("SETTINGBINDADDRESS what could possible have gone wrong?");
              }
              break;

            case CONNECTINGTODEVICE:
              if(incoming.startsWith("OK")){
                Serial.println("Successfully connected to "+currentDeviceName);
                //WE HAVE MOST PROBABLY LEFT AT MODE AND ARE IN COMMUNICATION MODE NOW
                PROGRAMSTATE = LISTENNMEA;
              }
              else if(incoming.startsWith("FAIL")){
                Serial.println("Failed to connect to "+currentDeviceName + ". Resetting device...");
                resetAllVariables();
                HC05_MODE = AT_MODE;            
                Set_HC05_MODE();
              }
              else{
                Serial.println("Error attempting connection to "+currentDeviceName + ". Resetting device...");
                resetAllVariables();
                HC05_MODE = AT_MODE;            
                Set_HC05_MODE();
              }
              break;
          }
        }
        else{
          switch(PROGRAMSTATE){
            case INQUIRINGDEVICES:
              if(incoming.startsWith("OK")){
                Serial.println("Finished inquiring devices.");
                currentDeviceIdx=0;
                if(deviceCount > 0){
                  currentDeviceAddr = devices[currentDeviceIdx];
                  ConfrontUserWithDevice(currentDeviceAddr);  
                }        
                else{
                  Serial.print("Sorry, I have not found any useful devices. Trying again.");
                  InitiateInquiry();
                }
              }
              else if(incoming.startsWith("+INQ")){
                int idx = incoming.indexOf(':');
                int idx2 = incoming.indexOf(',');
                String addr = incoming.substring(idx+1,idx2);
                addr.replace(':',',');
                addr.trim();
                devices[deviceCount] = addr;
                deviceCount++;
              }           
              break;
            case CONFRONTINGUSER:
              if(incoming.startsWith("+RNAME")){                    
                currentDeviceName = incoming.substring(incoming.indexOf(':')+1);
                
                flushOkString();
                
                Serial.println("Would you like to connect to '" + currentDeviceName + "'? Please type Y or N.");
              }
              else if(incoming.startsWith("FAIL")){
                Serial.println("Could not retrieve name of detected device... continuing to next");
                currentDeviceIdx++;
                if(currentDeviceIdx < deviceCount){
                  currentDeviceAddr = devices[currentDeviceIdx];
                  ConfrontUserWithDevice(currentDeviceAddr);
                }
                else{
                  Serial.println("Devices are exhausted. Searching again.");
                  InitiateInquiry();
                }
              }
              else{
                Serial.println("There was an error retrieving the device name... continuing to next");
                currentDeviceIdx++;
                if(currentDeviceIdx < deviceCount){
                  currentDeviceAddr = devices[currentDeviceIdx];
                  ConfrontUserWithDevice(currentDeviceAddr);
                }
                else{
                  Serial.println("Devices are exhausted. Searching again.");
                  InitiateInquiry();
                }
              }
              break;  
          }
        }
        
      }
    }
    //incoming = Serial1.readStringUntil('\n');
    //Serial.println(incoming);
    
    //Serial.write(inc);
  }
  
}

void Set_HC05_MODE(){
  if(currentFunctionStep==0){
    SETTINGHC05MODE = true;
    Serial.print("Now setting HC-05 mode to ");
    if(HC05_MODE == COMMUNICATION_MODE){
      Serial.println("COMMUNICATION_MODE");  
    }else if(HC05_MODE == AT_MODE){
      Serial.println("AT_MODE");
      PROGRAMSTATE = DO_ADCN;
    }
    digitalWrite(HC05_EN_PIN, LOW); //EN to LOW = disable (pull low to reset when changing modes!)
    currentFunctionStep++;
    dynamicEvent = t.after(500,Set_HC05_MODE);
  }
  else if(currentFunctionStep==1){
    digitalWrite(HC05_KEY_PIN, HC05_MODE); //KEY to HIGH = full AT mode, to LOW = communication mode
    currentFunctionStep++;
    dynamicEvent = t.after(500,Set_HC05_MODE);
  }
  else if(currentFunctionStep==2){
    digitalWrite(HC05_EN_PIN, HIGH); //EN to HIGH = enable
    currentFunctionStep++;
    if(HC05_MODE == AT_MODE){
      dynamicEvent = t.after(5000,Set_HC05_MODE);
    }
    else if(HC05_MODE == COMMUNICATION_MODE){
      dynamicEvent = t.after(5000,Set_HC05_MODE);
    }
  }
  else if(currentFunctionStep==3){
    if(HC05_MODE == AT_MODE){
      currentFunctionStep++;
      INITIALIZING = true;
      Serial.println(">>AT+ROLE=1");
      Serial1.println("AT+ROLE=1");
      dynamicEvent = t.after(1000,Set_HC05_MODE);
    }
    else if(HC05_MODE == COMMUNICATION_MODE){
      currentFunctionStep=0;    
      SETTINGHC05MODE = false;      
    }
  }  
  else if(currentFunctionStep==4){
    currentFunctionStep++;
    Serial.println(">>AT+CMODE=1");
    Serial1.println("AT+CMODE=1");
    dynamicEvent = t.after(1000,Set_HC05_MODE);    
  }
  else if(currentFunctionStep==5){
    currentFunctionStep++;
    Serial.println(">>AT+IPSCAN=1024,512,1024,512");
    Serial1.println("AT+IPSCAN=1024,512,1024,512");
    dynamicEvent = t.after(1000,Set_HC05_MODE);        
  }
  else if(currentFunctionStep==6){
    currentFunctionStep++;
    Serial.println(">>AT+INQM=1,15,12");
    Serial1.println("AT+INQM=1,15,12");
    dynamicEvent = t.after(1000,Set_HC05_MODE);        
  }
  else if(currentFunctionStep==7){
    currentFunctionStep++;
    Serial.println(">>AT+PSWD=0000");
    Serial1.println("AT+PSWD=0000");
    dynamicEvent = t.after(1000,Set_HC05_MODE);        
  }
  else if(currentFunctionStep==8){
    currentFunctionStep=0;    
    INITIALIZING=false;
    SETTINGHC05MODE = false;
  }
}

int Check_HC05_STATE(){
  Serial.println("Checking HC_05 connected state...");
  int state = digitalRead(HC05_STATE_PIN);
  if(state != HC05_OLDSTATE){
    HC05_OLDSTATE = state;
    if(state == CONNECTED){
      Serial.println("HC-05 is connected to a bluetooth device.");
    }
    else if(state == DISCONNECTED){
      Serial.println("HC-05 is disconnected from all bluetooth devices.");
    }
    digitalWrite(LED, state);
  }
  return state;  
}

void CountRecentAuthenticatedDevices(){
  PROGRAMSTATE = COUNTINGRECENTDEVICES;
  Serial.println("Now counting recent connected devices...");
  Serial.println("->AT+ADCN");
  Serial1.println("AT+ADCN");  
}

void CheckMostRecentAuthenticatedDevice(){
  PROGRAMSTATE = COUNTEDRECENTDEVICES;
  Serial.println("Now checking the most recent authenticated device...");
  Serial.println("->AT+MRAD");
  Serial1.println("AT+MRAD");  
}

void SearchAuthenticatedDevice(String addr){
  PROGRAMSTATE = SEARCHAUTHENTICATEDDEVICE;
  Serial.println("Now preparing to link to device whose address is: <" + addr + ">");
  Serial.println("->AT+INIT");
  Serial1.println("AT+INIT");  
}

void ConnectRecentAuthenticatedDevice(String addr){
  PROGRAMSTATE = CONNECTINGRECENTDEVICE;
  Serial.println("Now connecting to device whose address is: <" + addr + ">");
  Serial.println("->AT+LINK="+addr);
  Serial1.println("AT+LINK="+addr);  
}

int SetConnectionMode(int mode){
  PROGRAMSTATE = SETTINGCONNECTIONMODE;
  Serial.println("Setting connection mode...");
  Serial.println("->AT+CMODE="+String(mode));
  Serial1.println("AT+CMODE="+String(mode));  
  return mode;   
}

void InitiateInquiry(){
  PROGRAMSTATE = INITIATINGINQUIRY;
  Serial.println("Initiating inquiry...");
  Serial.println("->AT+INIT");
  Serial1.println("AT+INIT");  
}

void InquireDevices(){
  PROGRAMSTATE = INQUIRINGDEVICES;
  Serial.println("Inquiring devices...");
  Serial.println("->AT+INQ");
  Serial1.println("AT+INQ");  
}

void ConfrontUserWithDevice(String devicexAddr){
  PROGRAMSTATE = CONFRONTINGUSER;
  Serial.println("Retrieving name of device whose address is "+devicexAddr);
  Serial.println("->AT+RNAME "+devicexAddr);  
  Serial1.println("AT+RNAME "+devicexAddr);  
}

void SetBindAddress(){
  PROGRAMSTATE = SETTINGBINDADDRESS;
  Serial.println("Setting bind address to '" + currentDeviceName + "'s address <" + currentDeviceAddr + ">");
  Serial.println("->AT+BIND="+currentDeviceAddr);
  Serial1.println("AT+BIND="+currentDeviceAddr);
}

void LinkToCurrentDevice(String devicexAddr){
  PROGRAMSTATE = CONNECTINGTODEVICE;
  Serial.println("Very well! Connecting to '" + currentDeviceName + "'");
  Serial.println("->AT+LINK="+devicexAddr);
  Serial1.println("AT+LINK="+devicexAddr);
}

void flushOkString(){
  byte inc2 = Serial1.read();
  if(inc2 != -1){
    String okstring = char(inc2) + Serial1.readStringUntil('\n');
    Serial.println(okstring);
  }  
}

void resetAllVariables(){
  SETTINGHC05MODE = false;
  //INITIALSTATECHECK = true; //not this one, this is only for first time check...
  //DO_ADCN = false;
  //LISTENNMEA = false;
  //COUNTINGRECENTDEVICES = false;
  //COUNTEDRECENTDEVICES = false;
  //SEARCHAUTHENTICATEDDEVICE = false;
  //CONNECTINGRECENTDEVICE = false;
  //SETTINGCONNECTIONMODE = false;
  //INITIATINGINQUIRY = false;
  //INQUIRINGDEVICES = false;
  //CONFRONTINGUSER = false;
  //SETTINGBINDADDRESS = false;
  //CONNECTINGTODEVICE = false;
  INITIALIZING = false;
  deviceCount = 0;
  recentDeviceCount = 0;
  currentDeviceIdx = 0;
  currentFunctionStep = 0;
  for(int i=0;i<MAX_DEVICES;i++){
    devices[i] = "";
  }
  currentDeviceAddr = "";
  currentDeviceName = "";
}


String getErrorMessage(String errCodeStr){
  char errCodeHex[2];
  errCodeStr.toCharArray(errCodeHex,2);
  long errCode = strtol(errCodeHex,NULL,16);
  return ERRORMESSAGE[errCode];
}

/*  
 *  Copyright (c) 2016 John Romano D'Orazio john.dorazio@cappellaniauniroma3.org
 *   
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
