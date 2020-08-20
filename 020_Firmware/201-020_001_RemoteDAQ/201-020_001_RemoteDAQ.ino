/* Revision history
0.1 Initial version - ported from RocTrak project 
*/
#include <SPIFFS.h>
#include <SimpleCLI.h>
#include <FuGPS.h>
#include <Arduino.h>
#include <HardwareSerial.h>

#include <analogWrite.h>
#include <WiFi.h>
#include "Microchip_PAC193x.h"
#include <Wire.h>
// includes relating to APRS and packet radio

// #defines

#define hw_ver            1

//-------------------------------------
// fileLogger states
#define st_log_initial 10
#define st_log_standby 20
#define st_log_create_file 30
#define st_log_waiting_trigger 40
#define st_log_check_APRS 45
#define st_log_write 50
#define st_log_close_file 60
#define st_log_delete_files 70
#define st_log_error_state  80
int logState, nextLogState;

//--------------------------------------
// Define I/O
/* RocTrak(ino) rev0.1
  Designed to work with board 200_010_010-001_RocTrak rev0.1
  A few issues have been discovered with the board to date, 
  requiring some mods and use of pins differently to originially
  intended.
  - Radio PTT was not connected to the ESP32 - need to use Ch4output to pull PTT low 
  - VMon analog inputs not usable with WiFi activated - Did not read this until
    too late ie that when WiFi module is enabled, ADC2 is not reliable. Voltage monitoring
    will need to be performed using SmartBatteryIsolator and I2C interface
  - DTMF ArmStatus LED was connected to pin GPI36, which unfortunately is 1 of only 4 pins that can be an input only.
    Will connect this to one of the VMon pins instead, VMon3 
*/
//-----------------------------------------------------------
// Pin definitions going around the board 

    #define RX1Pin      36  // Connected to GPS Tx 
    #define DTMF_ValidPin 39
    #define DTMF_D3Pin  34
    #define DTMF_D2Pin  35
    #define DTMF_D1Pin  32
    #define DTMF_D0Pin  33
    #define TonePin     25   // audio tone out to radio for APRS or similar
    #define Tone2Pin    26   // 2nd/alternative audio tone out pin
    #define ToneInPin   27   // Analog input for audio signals
    #define Cont3Pin    14   // output ch3 continuity measurement
    #define Cont2Pin    12   // output ch2 continuity measurement
    #define Cont1Pin    13   // output ch1 continuity measurement
    #define GOut4Pin    15
    #define GOut3Pin    2  // output ch3 FET gate drive
    #define GOut2Pin    0  // output ch2 FET gate drive
    #define GOut1Pin    4   // output ch1 FET gate drive
    #define RX2Pin      16 // RX2, Connects to radio Uart Tx
    #define TX2Pin      17 // TX2, Connects to radio Uart Rx
    #define DTMF_ArmStatusPin 5 // was VMON input
    #define PTT4Pin     18   // PTT gate drive
    #define TX1Pin      19    // Connected to GPS Rx
    #define SDAPin      21
    #define SCLPin      22
    #define BuzzDrivePin 23 // Drives buzzer via Nch FET



// forward declarations

void errorCallback(cmd_error* e);
void cmdFileListCallback(cmd* c);
void cmdFileDownloadCallback(cmd* c);
void cmdFileEraseCallback(cmd* c);
void cmdCommandListCallback(cmd* c);
void cmdSystemStatusCallback(cmd* c);
void cmdOutputDriveCallback(cmd* c);
void updateFileSystemParameters(void);

struct FileSystemParamsType {
  unsigned long totalBytes;
  unsigned long usedBytes;
  unsigned long remainingBytes;
  float usedSpace;
  float remainingSpace;
} FileSystemParams;


// CLI objects

// Create CLI Object
SimpleCLI cli;

// Commands
Command cmdFileList;
Command cmdFileDownload;
Command cmdFileErase;
Command cmdParameterList;
Command cmdCommandList;
Command cmdGpsStatus;
Command cmdDtmfSimulate;
Command cmdSystemStatus;
Command cmdOutputDrive;
Command cmdPowerMon;
//----------------------------------------------------------------------------

Microchip_PAC193x PAC(10000,10000,10000,10000,4) ;  // PAC1934 power monitor, construct with new resistor value of 10000 uOhm

int value;

//--------------------------------------
// DTMF related vars
unsigned int DtmfVal;
unsigned int DtmfNew;
int          DtmfValArrayFull;
unsigned int DtmfValArray[25];
int          DtmfValCount;
int          DtmfState, DtmfNextState;
unsigned long DtmfTimer;

//--------------------------------------
// Output channel vars
int Cont1, Cont2, Cont3, Cont4;
String sCont1, sCont2, sCont3, sCont4;
int Out1, Out2, Out3, Out4;
String sOut1, sOut2, sOut3, sOut4;
//--------------------------------------
// Generic time related vars
unsigned long TimeDelta;

//--------------------------------------------------------------------------
// Webserver related vars & objects

const char* ssid     = "RocTrak_WAP";    // Replace with your network credentials
const char* password = "123456789";

WiFiServer server(80);  // Set web server port number to 80
//---------------------------------------------------------------------------

String header;          // Variable to store the HTTP request
String strGpsLockState;  //
String strGPSLat;  
String strGpsLong;
String strAltitude;
String strSatellites;
//--------------------------------------------------------------------------
// converts character array 
// to string and returns it 
String convertToString(char* a, int lengthOf) 
{ 
    int i; 
    String s = ""; 
    for (i = 0; i < lengthOf; i++) { 
        s = s + a[i]; 
        if (a[i]==0) break;
    } 
    return s; 
} 
//--------------------------------------------------------------------------
void updateContinuity(void){
if (digitalRead(Cont1Pin)==LOW) {
    sCont1 = "CC";
    Cont1 = 1;
  }
  else {
    sCont1 = "OC";
    Cont1=0;
  }
  if (digitalRead(Cont2Pin)==LOW) {
    sCont2 = "CC";
    Cont2 = 1;
  }
  else {
    sCont2 = "OC";
    Cont2=0;
  }if (digitalRead(Cont3Pin)==LOW) {
    sCont3 = "CC";
    Cont3 = 1;
  }
  else {
    sCont3 = "OC";
    Cont3=0;
  }/*if (digitalRead(Cont4Pin)==LOW) {
    sCont4 = "CC";
    Cont4 = 1;
  }
  else {
    sCont4 = "OC";
    Cont4=0;
  } */
}
  
void updateOutputs(void){

  if (Out1==1) sOut1 = "1";
  else sOut1 = "0";
  if (Out2==1) sOut2 = "1";
  else sOut2 = "0";
  if (Out3==1) sOut3 = "1";
  else sOut3 = "0";
  if (Out4==1) sOut4 = "1";
  else sOut4 = "0";
}

void setup() {
  // put your setup code here, to run once:

  // Setup I/O 
     
  // FET driven Output channels related
  pinMode(GOut1Pin, OUTPUT);
  pinMode(GOut2Pin, OUTPUT);
  pinMode(GOut3Pin, OUTPUT);
  pinMode(PTT4Pin, OUTPUT);
  pinMode(Cont1Pin, INPUT_PULLUP);
  pinMode(Cont2Pin, INPUT_PULLUP);
  pinMode(Cont3Pin, INPUT_PULLUP);
  pinMode(Cont4Pin, INPUT_PULLUP);
  

  // Others
  pinMode(BuzzDrivePin, OUTPUT);
  pinMode(TonePin, OUTPUT);
  pinMode(RX1Pin, INPUT);
  
  // Serial ports  
  Serial.begin(115200);     // Default port with USB converter
  
  // initialise SPIFFs
  if(!SPIFFS.begin(true)){
     Serial.println("An Error has occurred while mounting SPIFFS");
  }
  
  value=0;

  Wire.begin(SDA, SCL, 400000);

  // Define tasks
  TaskHandle_t TaskH_WebServer;
  TaskHandle_t TaskH_FileLogger;

    
  //----------------------------------------------------------------------------------------------------------
  // Setup and creation of tasks
  //----------------------------------------------------------------------------------------------------------

  //create WebServer task that will be executed in the Task_WebServer() function, with priority 5 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task_WebServer, // Task function. 
                    "WebServer",  // name of task. 
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    5,           // priority of the task 
                    &TaskH_WebServer,  // Task handle to keep track of created task 
                    0);          // pin task to core 0 
  delay(500); 
  
  //create FileLogger task that will be executed in the Task_Logger() function, with priority 7 and executed on core 0
  xTaskCreatePinnedToCore(
                    Task_FileLogger,   // Task function. 
                    "FileLogger",     // name of task. 
                    10000,       // Stack size of task 
                    NULL,        // parameter of the task 
                    7,           // priority of the task 
                    &TaskH_FileLogger,  // Task handle to keep track of created task 
                    0);          // pin task to core 0 
  delay(500); 

  // Turn on interrupts
  interrupts();
  
  //--------------------------------------------------------------
  // Initialise command line interpreter commands

  cmdCommandList = cli.addSingleArgCmd("?", cmdCommandListCallback);
  cmdFileList = cli.addSingleArgCmd("ls", cmdFileListCallback);
  cmdFileDownload = cli.addSingleArgCmd("dl", cmdFileDownloadCallback);
  cmdFileErase = cli.addSingleArgCmd("erase", cmdFileEraseCallback);
  cmdGpsStatus = cli.addSingleArgCmd("gps", cmdGpsStatusCallback);
  cmdDtmfSimulate = cli.addBoundlessCmd("dtmf", cmdDtmfSimulateCallback);
  cmdSystemStatus = cli.addSingleArgCmd("??", cmdSystemStatusCallback);
  cmdOutputDrive = cli.addSingleArgCmd("out", cmdOutputDriveCallback);
  cmdPowerMon = cli.addSingleArgCmd("pwr", cmdPowerMonitorCallback);
  
  cli.setOnError(errorCallback); // Set error Callback

  //--------------------------------------------------------------
  
  //--------------------------------------------------------------
}
//--------------------------------------------------------------------------
void loop() {
  // put your main code here, to run repeatedly:
  /*
  value+=5;
  analogWrite(PTT4Pin, value);
  delay(200);
  if (value>250) value =0;
  */
  updateContinuity();
  updateOutputs();
  
  /*
  digitalWrite(PTT4Pin, LOW);
  delay(500);
  digitalWrite(PTT4Pin, HIGH);
  delay(500);
  Serial.print("I love you dad\t");
  */

  // example code from fuGPS
  // Valid NMEA message
  
  //---------------------------
  if (Serial.available()){
    // basic command interpreter, primarily for SPIFFs functions at this time
    // Read out string from the serial monitor
    String input = Serial.readStringUntil('\n');
    // Parse the user input into the CLI
       cli.parse(input);
    }

    if (cli.errored()) {
        CommandError cmdError = cli.getError();

        Serial.print("ERROR: ");
        Serial.println(cmdError.toString());

        if (cmdError.hasCommand()) {
            Serial.print("Did you mean \"");
            Serial.print(cmdError.getCommand().toString());
            Serial.println("\"?");
        }
    }    

} // end of default loop()

//--------------------------------------------------------------------------
// ISRs
//
//---------------------------------------------------------------------------
  //---------------------------------------------------------------------------
  // 
  //---------------------------------------------------------------------------
  void Task_WebServer( void * parameter)
  {
    // Connect to Wi-Fi network with SSID and password
    Serial.print("Setting AP (Access Point)…");
    // Remove the password parameter, if you want the AP (Access Point) to be open
    WiFi.softAP(ssid, password);
    IPAddress IP = WiFi.softAPIP();
    Serial.print("AP IP address: ");
    Serial.println(IP);
    server.begin();   
    
    for(;;){
       
       
       WiFiClient client = server.available();   // Listen for incoming clients

  if (client) {                             // If a new client connects,
    Serial.println("New Client.");          // print a message out in the serial port
    String currentLine = "";                // make a String to hold incoming data from the client
    while (client.connected()) {            // loop while the client's connected
      if (client.available()) {             // if there's bytes to read from the client,
        char c = client.read();             // read a byte, then
        Serial.write(c);                    // print it out the serial monitor
        header += c;
        if (c == '\n') {                    // if the byte is a newline character
          // if the current line is blank, you got two newline characters in a row.
          // that's the end of the client HTTP request, so send a response:
          if (currentLine.length() == 0) {
            // HTTP headers always start with a response code (e.g. HTTP/1.1 200 OK)
            // and a content-type so the client knows what's coming, then a blank line:
            client.println("HTTP/1.1 200 OK");
            client.println("Content-type:text/html");
            client.println("Connection: close");
            client.println();
                                
            // Display the HTML web page
            client.println("<!DOCTYPE html><html>");
            client.println("<head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">");
            client.println("<link rel=\"icon\" href=\"data:,\">");
            // CSS to style the on/off buttons 
            // Feel free to change the background-color and font-size attributes to fit your preferences
            client.println("<style>html { font-family: Helvetica; display: inline-block; margin: 0px auto; text-align: center;}");
            client.println(".button { background-color: #4CAF50; border: none; color: white; padding: 16px 40px;");
            client.println("text-decoration: none; font-size: 30px; margin: 2px; cursor: pointer;}");
            client.println(".button2 {background-color: #555555;}");
            client.println("table, th, td {  border: 1px solid black;}</style></head>");
            
            // Web Page Heading
            //client.println("<body><h1>ESP32 Web Server</h1>");
            client.println("<body><h1>RocTrak - live status</h1>");
            //  client.println("");
            client.println("<h2>GPS status</h2>");
            client.print("<hr>");
            // Display current state of GPS Lock
            if (fuGPS.hasFix() == true) client.print("<p>GPS lock status : LOCK<br>");
            else client.print("<p>GPS lock status : NO LOCK<br>");
            client.print("# satellites: " + String(fuGPS.Satellites, 4) +"<br>");
            client.print("Altitude (above sea level): " + String(fuGPS.Altitude, 6) + "<br>");
            client.print("Location (dec deg): " + String(fuGPS.Latitude, 6) + ", " + String(fuGPS.Longitude, 6)+ "<br>");            
            client.print("<hr>");
            client.println("<h2>Power Supply</h2>");
            client.println("<table style=\"width:100%\">");
            client.println("<tr><th> </th><th>Supply1</th><th>Supply2</th><th>Supply3</th></tr>");
            client.println("<tr><td>Voltages (mV)</td><td>"+ String(PAC.Voltage[0],0)+"</td><td>"+ String(PAC.Voltage[1],0) + "</td><td>"+ String(PAC.Voltage[2],0) + "</td></tr>");
            client.println("<tr><td>Current (mA)</td><td>"+ String(PAC.Current[0],1)+"</td><td>"+ String(PAC.Current[1],1) + "</td><td>"+ String(PAC.Current[2],1) + "</td></tr>");
            client.println("<tr><td>Power (mW)</td><td>"+ String(PAC.Power[0],2)+"</td><td>"+ String(PAC.Power[1],2) + "</td><td>"+ String(PAC.Power[2],2) + "</td></tr>");
            client.println("</table>");
            client.println("<h2>Outputs</h2>");
            client.println("<table style=\"width:100%\">");
            client.println("<tr><th> </th><th>Ch1</th><th>Ch2</th><th>Ch3</th><th>Ch4</th></tr>");            
            client.println("<tr><td>Continuity</td><td>"+ sCont1 +"</td><td>"+ sCont2 + "</td><td>"+ sCont3 + "</td><td>"+ sCont4 +"</td></tr>");
            client.println("<tr><td>Outputs </td><td>"+ sOut1 +"</td><td>"+ sOut2 + "</td><td>"+ sOut3 + "</td><td>"+ sOut4 +"</td></tr>");
            client.println("</table>");
            client.println("</body></html>");

            
            
            // The HTTP response ends with another blank line
            client.println();
            // Break out of the while loop
            break;
          } else { // if you got a newline, then clear currentLine
            currentLine = "";
          }
        } else if (c != '\r') {  // if you got anything else but a carriage return character,
          currentLine += c;      // add it to the end of the currentLine
        }
      }
    }
    // Clear the header variable
    header = "";
    // Close the connection
    client.stop();
    Serial.println("Client disconnected.");
    Serial.println("");
  }

     
     delay(5);
     //yield();
    }
   
  }
  

  //------------------------------------------------------------------------------
  // Task_FileLogger : state machine to manage logging of flight variables and 
  // measurements to file
  //------------------------------------------------------------------------------
  void Task_FileLogger( void * parameter)
  {
  // logger writes a single file line in ascii format, once per second.
  // format
  // time (ms)  (from free running counter)
  // time (HMS) (from GPS time)
  // #satellites
  // fix quality
  // accuracy (m)
  // lattitude  (decimal degrees)
  // longitude  (decimal degrees)
  // altitude (m)
  // supply voltage 1 to 3
  // supply current 1 to 3
  // continuity 1 to 4
  // output state 1 to 4
  // DTMF arm status
  // DTMF command

  unsigned long loggerTimer;
  String logEntry;
  String tempString;
  String fileName;
  char tempCharArray[80];
  int i;

  logState = st_log_initial;
  nextLogState = st_log_initial;
  File logFile;

  for(;;){

    switch (logState) {

      case st_log_initial :
          // Check there is enough space to log something, update global FS parameters with result
          updateFileSystemParameters();
          if (FileSystemParams.remainingBytes > 300000){
            logState = st_log_standby;
            Serial.println("->st_log_standby");
          }
          else {
            // need to make some space, for now just bum out
            logState = st_log_error_state;
            Serial.println("not enough remaining space: " + String(FileSystemParams.remainingBytes));
            Serial.println("->st_log_error_state");
            
          }
      break;
      
      case st_log_standby :
        // wait for GPS lock AND APRS rapid transmission rate
        if ((fuGPS.hasFix())&&(myAPRS.enableRapidPackets==1)){
          // create file name from date and time, mdhms (month+day+hour+minute+seconds all 2 digit strings)
          fileName = "/log_";
          // extract time from GPS, 
          myAPRS.setTime(fuGPS.Months, fuGPS.Days, fuGPS.Hours, fuGPS.Minutes, fuGPS.Seconds);
          fileName += convertToString(myAPRS.mdhmsTime, sizeof(myAPRS.mdhmsTime));
          fileName += ".txt";
          Serial.println(fileName);
          logState = st_log_create_file;
          Serial.println("->st_log_create_file");
          
        }
      
      
      break;
      
      case st_log_create_file :
          logFile = SPIFFS.open(fileName, FILE_WRITE);      

          if (!logFile) {
            Serial.println("unable to create logfile");  
            logState = st_log_error_state;
            Serial.println("->st_log_error");            
          }
          else {
            logState = st_log_waiting_trigger;
            loggerTimer = millis();
            Serial.println("->st_log_waiting_trigger");   
            // write file header
            logEntry ="time(ms), time(hms),#Satellites, Qual, HDOP,lat, long, alt, speed, Course, v1, v2, v3, I1, I2, I3, cont1, cont2,cont3, cont4, out1, out2, out3, out4, dmf_state, dtmf0, dtmf1, dtmf2, dtmf3";
            logFile.println(logEntry);          
          }
      
      break;

      case st_log_waiting_trigger :
          
          if ((loggerTimer + 960 < millis())||(DtmfNew==1)){ 
               loggerTimer = millis();
              // Update readings from PAC
              PAC.UpdateVoltage();
              PAC.UpdateVsense();
              PAC.UpdateCurrent();
              PAC.UpdatePower();
    
              // generate file row
              logEntry = String(millis());
    
              // use APRS class functions to create location and time strings from current GPS result
              myAPRS.setLocationFromDecDeg((float)fuGPS.Latitude, (float)fuGPS.Longitude, (float)fuGPS.Altitude);
              // extract time from GPS, 
              myAPRS.setTime(fuGPS.Months, fuGPS.Days, fuGPS.Hours, fuGPS.Minutes, fuGPS.Seconds);
              
              logEntry += ",";
              logEntry += convertToString(myAPRS.hmsTime, sizeof(myAPRS.hmsTime));
              logEntry += ",";
              logEntry += String(fuGPS.Satellites);
              logEntry += ",";
              logEntry += String(fuGPS.Quality);
              logEntry += ",";
              logEntry += String(fuGPS.Accuracy, 3);
              logEntry += ",";
              logEntry += String(fuGPS.Latitude, 6); //convertToString(myAPRS.sLattitude, sizeof(myAPRS.sLattitude));                    
              logEntry += ",";
              logEntry += String(fuGPS.Longitude, 6); //convertToString(myAPRS.sLongitude, sizeof(myAPRS.sLongitude));                    
              logEntry += ",";
              logEntry += convertToString(myAPRS.sAltitude, sizeof(myAPRS.sAltitude));                    
              logEntry += ",";
              logEntry += String(fuGPS.Speed);
              logEntry += ",";
              logEntry += String(fuGPS.Course);
              logEntry += ",";              
              // Generate status string. Combine GPS, power monitoring and Output channel status
              logEntry += String(PAC.Voltage[0],0)+ "," + String(PAC.Voltage[1],0) + "," + String(PAC.Voltage[2],0) + "," + String(PAC.Current[0],1) + "," + String(PAC.Current[1],1) + "," + String(PAC.Current[2],1) + ",";
              logEntry += sCont1 + "," + sCont2 + "," + sCont3 + "," + sCont4 + "," + sOut1 + "," + sOut2 + "," + sOut3 + "," + sOut4 + ",";
              logEntry += String(DtmfState);
              logEntry += ",";
              logEntry += String(DtmfValArray[0])+ "," + String(DtmfValArray[1])+ "," + String(DtmfValArray[2]) + "," + String(DtmfValArray[4]);
              logEntry += "\r\n";
              logState = st_log_check_APRS;
              Serial.println("->st_log_check_APRS");
         }
         else if (myAPRS.enableRapidPackets==0){
              // trigger to abort logging
              logState = st_log_close_file;
              Serial.println("->st_log_close_file");
         }
      break;

      case st_log_check_APRS :
          if (!digitalRead(PTT4Pin)) { // hold up here if radio Tx in progress
            logState = st_log_write;
            Serial.println("->st_log_write");
          }

      break;
      
      case st_log_write :
          logFile.print(logEntry);
          Serial.println(logEntry);

          updateFileSystemParameters();
          if (FileSystemParams.remainingBytes < 10000){
            logFile.close();
            Serial.println("closing logfile, not enough space remaining");
            logState = st_log_initial;
            Serial.println("->st_log_initial");
          }
          else {
            logState = st_log_waiting_trigger;
            Serial.println("->st_log_waiting_trigger");
          }
      break;
      
      case st_log_close_file :
          logFile.close();
          Serial.println("closing logfile");
          logState = st_log_standby;
          Serial.println("->st_log_standby");
      break;
      
      case st_log_delete_files :
        logState = st_log_initial;
        nextLogState = st_log_initial;
      break;

      case st_log_error_state :
        logState = st_log_initial;
        nextLogState = st_log_initial;
      break;
      
      default:
        logState = st_log_initial;
        nextLogState = st_log_initial;
      break;
      
    }
    
    //logState = nextLogState;
    delay(25);
    yield();  

  }

  }  
  void errorCallback(cmd_error* e){

    CommandError cmdError(e); // Create wrapper object
    Serial.print("ERROR: ");
    Serial.println(cmdError.toString());

    if (cmdError.hasCommand()) {
        Serial.print("Did you mean \"");
        Serial.print(cmdError.getCommand().toString());
        Serial.println("\"?");
    }
  }


  void cmdCommandListCallback(cmd* c){
    
    // No arguments at all, just list the available commands
    Serial.println("-----------------------------------------------");
    Serial.println("\r\nCommand list");
    Serial.println("?       \t Command list");
    Serial.println("??      \t System status dump");   // to be implemented
    Serial.println("ls      \t List files (with index)");
    Serial.println("dl x    \t download file at index x");
    Serial.println("erase x \t erase file at index x");
    Serial.println("gps     \t gps status dump");
    Serial.println("dtmf x1 x2 x3 x4 dtmf simulation with boundless arguments");
    Serial.println("out  x  \t turn on out ch x for 1000 ms");   // to be implemented
    Serial.println("pwr     \t power monitor values");   // to be implemented
    Serial.println("-----------------------------------------------");
  }

  void cmdSystemStatusCallback(cmd* c){
    // dump everything we have in terms of system status
    Serial.println("System overall status");
    updateFileSystemParameters();
    Serial.println("-----------------------------------------------------------");
    Serial.println("\r\n SPIFFs status");
    Serial.println("Total bytes: " + String(FileSystemParams.totalBytes));
    Serial.println("Used bytes: " + String(FileSystemParams.usedBytes) +", "+ String(FileSystemParams.usedSpace) + " %");
    Serial.println("Remaining bytes: " + String(FileSystemParams.remainingBytes) + ", " + String(FileSystemParams.remainingSpace) + " %");
    Serial.println("-----------------------------------------------------------");
    screenDumpGPSStatus();
    Serial.println("-----------------------------------------------------------");
    screenDumpPowerStatus();
    Serial.println("-----------------------------------------------------------");
    Serial.println(" WiFi Status");
    Serial.print(" SSID: ");
    Serial.print(ssid);
    Serial.print(", pwd: ");
    Serial.println(password);
    Serial.println("-----------------------------------------------------------");
    Serial.println(" Radio settings");
    Serial.println("Tx Freq: " + String(RadioParams.TxFreq, 3));    
    Serial.println("Rx Freq: " + String(RadioParams.RxFreq, 3));
    Serial.println("Tx delay: " + String(RadioParams.TxDelay));
    Serial.println("-----------------------------------------------------------");
    Serial.println(" APRS settings");
    Serial.print("Call sign: ");
    Serial.println(myAPRS.mycall);
    Serial.println("ssid: " + String(myAPRS.myssid));
    Serial.println("Pre-amble length: " + String(myAPRS.preambleBits));
    if (myAPRS.enableRapidPackets == 1) Serial.println("rapid Tx: enabled");
    else Serial.println("rapid Tx: disabled");
    
    Serial.println("-----------------------------------------------------------");
    
  }
  void cmdOutputDriveCallback(cmd* c){
    // turn on output channel x for 1000ms
    Serial.println("this is the output command callback");
     int found=0;
    int i;
    int argValue;
    String fileName;
    Command cmd(c); // Create wrapper object
    // Get first (and only) Argument
    Argument arg = cmd.getArgument(0);
    // Get value of argument
    String argVal = arg.getValue();
    //Serial.println(cmd.getName() + " " + argVal);

    // find file by index
    // convert argument value from string to int
    argValue = (int)argVal.toInt();    
    switch (argValue){ 

      case 1: { 
        Serial.print("out1 : On");
        digitalWrite(GOut1Pin, 1);
        delay(1000);
        Serial.println("  Off");
        digitalWrite(GOut1Pin, 0);
        break;
      }
      case 2: { 
        Serial.print("out2 : On");        
        digitalWrite(GOut2Pin, 1);
        delay(1000);
        Serial.println("  Off");        
        digitalWrite(GOut2Pin, 0);
        break;
      }
      case 3: { 
        Serial.print("out3 : On");        
        digitalWrite(GOut3Pin, 1);
        delay(1000);
        Serial.println("  Off");
        digitalWrite(GOut3Pin, 0);
        break;
      }
      case 4: { 
        Serial.print("out4 : On");        
        digitalWrite(GOut4Pin, 1);
        delay(1000);
        Serial.println("  Off");
        digitalWrite(GOut4Pin, 0);
        break;
      }
    }
  }

  void cmdFileListCallback(cmd* c){
    
   File root = SPIFFS.open("/");
   File file = root.openNextFile();
   int i=0;
   Command cmd(c); // Create wrapper object
   // Get first (and only) Argument
   Argument arg = cmd.getArgument(0);
   // Get value of argument
   String argVal = arg.getValue();
   // Echo command
   Serial.println(cmd.getName() + " " + argVal);
      
   Serial.println("total bytes: " + String(SPIFFS.totalBytes()));
   Serial.println("used bytes: " + String(SPIFFS.usedBytes()));

   while(file){
       Serial.print("FILE " + String(i)+ " ");
      Serial.print(file.name());
      Serial.println(" " + String(file.size()));
      file = root.openNextFile();
      i++;
  }
    Serial.println("");
  }

  void cmdFileDownloadCallback(cmd* c){

    int found=0;
    int i;
    int argValue;
    String fileName;
    Command cmd(c); // Create wrapper object
    // Get first (and only) Argument
    Argument arg = cmd.getArgument(0);
    // Get value of argument
    String argVal = arg.getValue();
    Serial.println(cmd.getName() + " " + argVal);

    // find file by index
    File root = SPIFFS.open("/");
    File file1 = root.openNextFile();
    found=0;
    i =0;
    // convert argument value from string to int
    argValue = (int)argVal.toInt();    
    
    while(file1){
      if (i== argValue){
        Serial.println("dumping " + String(file1.name()) + " " + String(file1.size()) + "bytes");
        fileName = file1.name();
        found=1;     
        break;
      }
      else {
      file1 = root.openNextFile();
      i++;
      }
    }

    if (found==1){
      while(file1.available()){
          Serial.write(file1.read());
      }
      Serial.println("");
    }
    file1.close();
    Serial.println("");
  }


  void cmdFileEraseCallback(cmd* c){

    int found=0;
    int i;
    int argValue;
    String fileName;
    Command cmd(c); // Create wrapper object
    // Get first (and only) Argument
    Argument arg = cmd.getArgument(0);
    // Get value of argument
    String argVal = arg.getValue();
    // Echo command
    Serial.println(cmd.getName() + " " + argVal);
    
     // find file by index
    File root = SPIFFS.open("/");
    File file1 = root.openNextFile();
    found=0;
    i =0;
    // convert argument value from string to int
    argValue = (int)argVal.toInt();    
    
    while(file1){
      if (i== argValue){
        Serial.println("erasing " + String(file1.name()) + " " + String(file1.size()) + "bytes");
        fileName = file1.name();
        found=1;     
        break;
      }
      else {
      file1 = root.openNextFile();
      i++;
      }
    }

    if (found==1){
      if(SPIFFS.remove(fileName)) Serial.println("File removed");
      else Serial.println("File unable to be removed");
    }
    else Serial.println("File unable to be removed");
   
 }
  void updateFileSystemParameters(void)
  {
  //  File root = SPIFFS.open("/");
     FileSystemParams.totalBytes = SPIFFS.totalBytes();
     FileSystemParams.usedBytes = SPIFFS.usedBytes();
     FileSystemParams.remainingBytes = FileSystemParams.totalBytes - FileSystemParams.usedBytes;
     FileSystemParams.usedSpace = 100.0*(float)FileSystemParams.usedBytes/(float)FileSystemParams.totalBytes;
     FileSystemParams.remainingSpace = 100.0*(float)FileSystemParams.remainingBytes/(float)FileSystemParams.totalBytes;
  }

 
