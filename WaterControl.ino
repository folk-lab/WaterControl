/*
Water Control
 */

#include <Ethernet.h>
#include <SPI.h>
#include <SD.h>
#include <P1AM.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <ArduinoJson.h>
#include "FlowSensor.h"

#define NUMPULSEINPUTS 8
#define TIMEOUTMICROS 2000000 //If more micros have elapsed since last pulse, set pulse length to default
#define DEFAULTMICROS 999999999
#define NUMFLOWSENSORS 4
//These default values are for GPM flow and DegF temp for VFS50-5-1001
/*
#define DEFAULTMAXFLOWSCALE 5.28
#define DEFAULTMINFLOWSCALE 0.0
#define DEFAULTFLOWFREQ 100.0
#define DEFAULTMAXTEMPSCALE 150.0
#define DEFAULTMINTEMPSCALE 14.0
#define DEFAULTTEMPFREQ 100.0
#define DEFAULTTIMEOUTMICROS 2000000
*/
//These default values are for LPM flow and DegC temp for VFS50-5-1001
#define DEFAULTMAXFLOWSCALE 20.0
#define DEFAULTMINFLOWSCALE 0.0
#define DEFAULTFLOWFREQ 100.0
#define DEFAULTMAXTEMPSCALE 65.56
#define DEFAULTMINTEMPSCALE -10.0
#define DEFAULTTEMPFREQ 100.0
#define DEFAULTTIMEOUTMICROS 3000000
volatile uint32_t lastpulseperiod[NUMPULSEINPUTS] = {0}; //pulse length in micros
volatile uint8_t newpulseflag[NUMPULSEINPUTS] = {0}; //Set to 1 when a new pulse arrives
// assign a MAC address for the Ethernet controller.
// fill in your address here:
byte mac[] = {
  0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0x99
};
// assign an IP address for the controller:
IPAddress ip(192, 168, 1, 20);

typedef struct FlowConfig
{
  String tag;
  String location;
  String flowunit;
  String tempunit;
  float maxflowscale;
  float minflowscale;
  float flowthresh;
  float flowfreq;
  float maxtempscale;
  float mintempscale;
  float tempthresh;
  float tempfreq;
  uint32_t timeoutmicros;
}FlowConfig;

// Our configuration structure.
//
// Never use a JsonDocument to store the configuration!
// A JsonDocument is *not* a permanent storage; it's only a temporary storage
// used during the serialization phase. See:
// https://arduinojson.org/v6/faq/why-must-i-create-a-separate-config-object/
struct Config {
  FlowConfig flowconfig[NUMFLOWSENSORS];
  //char hostname[64];
  //int port;
};



const char *filename = "/config.txt";  // <- SD library uses 8.3 filenames
Config config;                         // <- global configuration object

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);
EthernetServer mbServer(502);
ModbusTCPServer modbusTCPServer;

FlowSensor flowsensor[NUMFLOWSENSORS] = 
{
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 10.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 60.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS),
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 9.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 55.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS),
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 8.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 50.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS),
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 7.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 45.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS)
};

//FlowSensor flowsensor(4.2, 0, 100, 40, 140, 400, 2000000);

uint32_t lastMillis, nowMillis, nowMicros = 0;
uint32_t lastLoopTime = 0;
uint32_t longestLoopTime = 0;
uint32_t loopcount = 0;
uint8_t faultflag = 0;
void setup() {
  // You can use Ethernet.init(pin) to configure the CS pin
  //Ethernet.init(10);  // Most Arduino shields
  //Ethernet.init(5);   // MKR ETH Shield
  //Ethernet.init(0);   // Teensy 2.0
  //Ethernet.init(20);  // Teensy++ 2.0
  //Ethernet.init(15);  // ESP8266 with Adafruit FeatherWing Ethernet
  //Ethernet.init(33);  // ESP32 with Adafruit FeatherWing Ethernet

  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);
  
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);

  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);

  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
  
  // start the SPI library:
  SPI.begin();

  // start the Ethernet connection
  Ethernet.begin(mac, ip);

  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */

  // Check for Ethernet hardware present
  if (Ethernet.hardwareStatus() == EthernetNoHardware) {
    Serial.println("Ethernet shield was not found.  Sorry, can't run without hardware. :(");
    while (true) {
      delay(1); // do nothing, no point running without Ethernet hardware
    }
  }
  if (Ethernet.linkStatus() == LinkOFF) {
    Serial.println("Ethernet cable is not connected.");
  }

  // start listening for clients
  server.begin();
  mbServer.begin();
  // start the Modbus TCP server
  if (modbusTCPServer.begin()) 
  {
    Serial.println("Modbus TCP Server started");  
  }
  else
  {
    Serial.println("Failed to start Modbus TCP Server!");
    while (1);
  }
  // give the sensor and Ethernet shield time to set up:
  delay(1000);

  attachInterrupt(digitalPinToInterrupt(A1), pinA1int, FALLING);
  attachInterrupt(digitalPinToInterrupt(A2), pinA2int, FALLING);
  attachInterrupt(digitalPinToInterrupt(0), pin0int, FALLING);
  attachInterrupt(digitalPinToInterrupt(1), pin1int, FALLING);
  attachInterrupt(digitalPinToInterrupt(3), pin3int, FALLING);
  attachInterrupt(digitalPinToInterrupt(4), pin4int, FALLING);
  attachInterrupt(digitalPinToInterrupt(6), pin6int, FALLING);
  attachInterrupt(digitalPinToInterrupt(7), pin7int, FALLING);
  
  while (!P1.init())
  { 
    ; //Wait for Modules to Sign on   
  }
  //Configure watchdog to reset after 5000ms timeout
  P1.configWD(5000, TOGGLE);
  P1.startWD();
  // Initialize SD library
  const int chipSelect = SDCARD_SS_PIN;
  while (!SD.begin(chipSelect)) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);

  // Create configuration file
  Serial.println(F("Saving configuration..."));
  saveConfiguration(filename, config);
  display_freeram();
  // configure a holding register at address 0x00
  modbusTCPServer.configureHoldingRegisters(0x00, 200);
  for(int i = 0; i < 200; i++)
  {
    modbusTCPServer.holdingRegisterWrite(i, (i+1) * 256);
  }
  float testfloat = 12.3456;
  
  //uint16_t register1 = ((uint32_t)testfloat >> 16) & 0xFFFF;  // right shift >> by 16bits
  //uint16_t register2 = (uint32_t)testfloat & 0xFFFF;
  
  // float voltage value cast to 32bit int and split between 2x 16bit ints
  
  
  //uint32_t dualValue1 = *((uint32_t *)&testfloat);
  uint32_t *dualValue1 = (uint32_t *)&testfloat;
  //uint16_t *w1 = w2 + 2;
  //uint32_t dualValue1 = (uint32_t)testfloat;
  //uint32_t dualValue1 = (uint32_t)testfloat;
  uint16_t register1 = *dualValue1 >> 16;  // right shift >> by 16bits
  uint16_t register2 = *dualValue1 & 0xFFFF;
  //uint16_t register2 = dualValue1;
  //uint16_t register1 = *w1;  // right shift >> by 16bits
  //uint16_t register2 = *w2;  
  // write values to input registers for voltage value
  modbusTCPServer.holdingRegisterWrite(0x00, register1);
  modbusTCPServer.holdingRegisterWrite(0x01, register2);

}



void loop() {

  //Reset watchdog timer
  P1.petWD();
  nowMillis = millis();
  nowMicros = micros();
  lastLoopTime = nowMillis - lastMillis;
  lastMillis = nowMillis;
  faultflag = 0;
  for(int i = 0; i < NUMFLOWSENSORS; i++)
  {
    if(newpulseflag[i])
    {
      flowsensor[i].updateflow(lastpulseperiod[i]);
      newpulseflag[i] = 0;
    }
    if(newpulseflag[i + NUMFLOWSENSORS])//second half of inputs are for temperature
    {
      flowsensor[i].updatetemp(lastpulseperiod[i + NUMFLOWSENSORS]);
      newpulseflag[i + NUMFLOWSENSORS] = 0;
    }
    flowsensor[i].process();
    if(flowsensor[i].getflowfault() || flowsensor[i].gettempfault())
    {
      faultflag = 1;
    }
  }
  //If any channel faulted, switch outputs on
  if(faultflag)
  {
    P1.writeDiscrete(0xFF,1);  //Turn all slot 1 outputs on    
  }
  else
  {
    P1.writeDiscrete(0x00,1);  //Turn all slot 1 outputs off    
  }
  
  loopcount++;
  if(loopcount > 5000)
  {
    longestLoopTime = 0;
    loopcount = 0;
    /*
    for(int i = 0; i < NUMFLOWSENSORS; i++)
    {
      flowsensor[i].resetfault();

    }
    */

  }
  if(lastLoopTime > longestLoopTime)  
  {
    longestLoopTime = lastLoopTime;
  }

  // listen for incoming Ethernet connections:
  listenForEthernetClients();
  handlemodbus();
}

void handlemodbus() {
  // listen for incoming clients
  static bool accepted = false;
  EthernetClient mbclient = mbServer.available();
  
  if (mbclient) {
    // a new client connected
    if (accepted == false)
    {
      //Serial.println("new mbclient");
      // let the Modbus TCP accept the connection 
      modbusTCPServer.accept(mbclient);
      accepted = true;
    }
    
    

    if (mbclient.connected()) {
      // poll for Modbus TCP requests, while client connected
      modbusTCPServer.poll();
      //Serial.println("polling");
      //display_freeram();

    }

         // give the web browser time to receive the data
      //delay(1);
    // close the connection:
    //Serial.println("client disconnected"); 
//mbclient.stop();

   
  } 
  else
  {
    accepted = false;
  }
}

void listenForEthernetClients() {
  // listen for incoming clients
  EthernetClient client = server.available();
  String readString; 
  if (client) {
    //Serial.println("Got a client");
    // an HTTP request ends with a blank line
    bool currentLineIsBlank = true;
    while (client.connected()) {
      if (client.available()) {
        char c = client.read();
        // if you've gotten to the end of the line (received a newline
        // character) and the line is blank, the HTTP request has ended,
        // so you can send a reply
        if (readString.length() < 100) 
        {
          //store characters to string
          readString += c;
        }

        if (c == '\n' && currentLineIsBlank) {
          if (readString.indexOf("?buttonResetFault") >0 )
          {
            for(int i = 0; i < NUMFLOWSENSORS; i++)
            {
            flowsensor[i].resetfault();
            }   
          }
          // send a standard HTTP response header
          client.println("HTTP/1.1 200 OK");
          client.println("Content-Type: text/html");
          client.println("Connection: close");  // the connection will be closed after completion of the response
          if (readString.indexOf("?Autorefresh") >0 )
          {
            client.println("Refresh: 1");  // refresh the page automatically every 5 sec
          }
          client.println();
          client.println("<!DOCTYPE HTML>");
          client.println("<html>");          
          client.println();
          client.println("<a href=\"/?Autorefresh\"\">AutoRefresh</a>");
          client.println("<br />");
          client.println("<a href=\"/\"\">StopAutoRefresh</a>");
          client.println("<br />");
          client.println("<a href=\"/?buttonResetFault\"\">Reset Faults</a>");
          client.println("<br />");

          for(int i = 0; i < NUMFLOWSENSORS; i++)
          {
            client.print("Flow ");
            client.print(i);
            client.print(" Pulse period: ");
            client.print(flowsensor[i].getflowmicros());
            client.print("us");
            client.println("<br />");
            client.print("Flow ");
            client.print(i);
            client.print(" freq: ");
            client.print(flowsensor[i].getflowfreq());
            client.print("Hz");
            client.println("<br />");
            client.print("Flow ");
            client.print(i);
            client.print(" Scaled: ");
            client.print(flowsensor[i].getflowscaled());
            client.print("LPM");
            client.println("<br />");
            client.print("Flow ");
            client.print(i);
            client.print(" Fault: ");
            client.print(flowsensor[i].getflowfault());
            client.println("<br />");            
            client.print("Temp ");
            client.print(i);
            client.print(" Pulse period: ");
            client.print(flowsensor[i].gettempmicros());
            client.print("us");
            client.println("<br />");
            client.print("Temp ");
            client.print(i);
            client.print(" freq: ");
            client.print(flowsensor[i].gettempfreq());
            client.print("Hz");
            client.println("<br />");
            client.print("Temp ");
            client.print(i);
            client.print(" scaled: ");
            client.print(flowsensor[i].gettempscaled());
            client.print("DegC");
            client.println("<br />");
            client.print("Temp ");
            client.print(i);
            client.print(" Fault: ");
            client.print(flowsensor[i].gettempfault());
            client.println("<br />");
          }
          client.print("Millis: ");
          client.print(millis());
          client.println("<br />");
          client.print("LoopTime: ");
          client.print(lastLoopTime);
          client.println("<br />");
          client.print("longestLoopTime: ");
          client.print(longestLoopTime);
          client.println("<br />");
          client.print("loopcount: ");
          client.print(loopcount);
          client.println("<br />");
          break;
        }
        if (c == '\n') {
          // you're starting a new line
          currentLineIsBlank = true;
        } else if (c != '\r') {
          // you've gotten a character on the current line
          currentLineIsBlank = false;
        }
      }
    }
    // give the web browser time to receive the data
    delay(1);
    // close the connection:
    client.stop();
    
  
  }
}
// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  File file = SD.open(filename);

  // Allocate a temporary JsonDocument
  StaticJsonDocument<2048> doc;

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));
  int i = 0;
for (JsonObject item : doc.as<JsonArray>()) {

  config.flowconfig[i].tag = item["tag"].as<String>(); // "Flowsensor1", "Flowsensor2", "Flowsensor3", "Flowsensor4"
  config.flowconfig[i].location = item["location"].as<String>(); // "Compressor Room", "Compressor Room", "Compressor Room", ...
  config.flowconfig[i].flowunit = item["flowunit"].as<String>(); // "lpm", "lpm", "lpm", "lpm"
  config.flowconfig[i].tempunit = item["tempunit"].as<String>(); // "degC", "degC", "degC", "degC"
  config.flowconfig[i].maxflowscale = item["maxflowscale"]; // 20, 20, 20, 20
  config.flowconfig[i].minflowscale = item["minflowscale"]; // 0, 0, 0, 0
  config.flowconfig[i].flowthresh = item["flowthresh"]; // 9, 9, 9, 9
  config.flowconfig[i].flowfreq = item["flowfreq"]; // 100, 100, 100, 100
  config.flowconfig[i].maxtempscale = item["maxtempscale"]; // 65.56, 65.56, 65.56, 65.56
  config.flowconfig[i].mintempscale = item["mintempscale"]; // -10, -10, -10, -10
  config.flowconfig[i].tempfreq = item["tempfreq"]; // 100, 100, 100, 100
  config.flowconfig[i].timeoutmicros = item["timeoutmicros"]; // 3000000, 3000000, 3000000, 3000000
  Serial.println(config.flowconfig[i].tag);
  Serial.println(config.flowconfig[i].location);
  Serial.println(config.flowconfig[i].flowunit);
  Serial.println(config.flowconfig[i].tempunit);
  Serial.println(config.flowconfig[i].maxflowscale);
  Serial.println(config.flowconfig[i].minflowscale);
  Serial.println(config.flowconfig[i].flowthresh);
  Serial.println(config.flowconfig[i].flowfreq);
  Serial.println(config.flowconfig[i].maxtempscale);
  Serial.println(config.flowconfig[i].mintempscale);
  Serial.println(config.flowconfig[i].tempfreq);
  Serial.println(config.flowconfig[i].timeoutmicros);
  i++;
  if(i >= NUMFLOWSENSORS)
  {
  break;
  }
  
}

  // Close the file (Curiously, File's destructor doesn't close the file)
  display_freeram();
  file.close();
  display_freeram();
}

// Saves the configuration to a file
void saveConfiguration(const char *filename, const Config &config) {
  // Delete existing file, otherwise the configuration is appended to the file
  SD.remove(filename);

  // Open file for writing
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println(F("Failed to create file"));
    return;
  }

  // Allocate a temporary JsonDocument
StaticJsonDocument<2048> doc;

JsonObject doc_0 = doc.createNestedObject();
doc_0["tag"] = "Flowsensor1";
doc_0["location"] = "Compressor Room";
doc_0["flowunit"] = "lpm";
doc_0["tempunit"] = "degC";
doc_0["maxflowscale"] = 20;
doc_0["minflowscale"] = 0;
doc_0["flowthresh"] = 9;
doc_0["flowfreq"] = 100;
doc_0["maxtempscale"] = 65.56;
doc_0["mintempscale"] = -10;
doc_0["tempfreq"] = 100;
doc_0["timeoutmicros"] = 3000000;

JsonObject doc_1 = doc.createNestedObject();
doc_1["tag"] = "Flowsensor2";
doc_1["location"] = "Compressor Room";
doc_1["flowunit"] = "lpm";
doc_1["tempunit"] = "degC";
doc_1["maxflowscale"] = 20;
doc_1["minflowscale"] = 0;
doc_1["flowthresh"] = 9;
doc_1["flowfreq"] = 100;
doc_1["maxtempscale"] = 65.56;
doc_1["mintempscale"] = -10;
doc_1["tempfreq"] = 100;
doc_1["timeoutmicros"] = 3000000;

JsonObject doc_2 = doc.createNestedObject();
doc_2["tag"] = "Flowsensor3";
doc_2["location"] = "Compressor Room";
doc_2["flowunit"] = "lpm";
doc_2["tempunit"] = "degC";
doc_2["maxflowscale"] = 20;
doc_2["minflowscale"] = 0;
doc_2["flowthresh"] = 9;
doc_2["flowfreq"] = 100;
doc_2["maxtempscale"] = 65.56;
doc_2["mintempscale"] = -10;
doc_2["tempfreq"] = 100;
doc_2["timeoutmicros"] = 3000000;

JsonObject doc_3 = doc.createNestedObject();
doc_3["tag"] = "Flowsensor4";
doc_3["location"] = "Compressor Room";
doc_3["flowunit"] = "lpm";
doc_3["tempunit"] = "degC";
doc_3["maxflowscale"] = 20;
doc_3["minflowscale"] = 0;
doc_3["flowthresh"] = 9;
doc_3["flowfreq"] = 100;
doc_3["maxtempscale"] = 65.56;
doc_3["mintempscale"] = -10;
doc_3["tempfreq"] = 100;
doc_3["timeoutmicros"] = 3000000;

  // Serialize JSON to file
  if (serializeJson(doc, file) == 0) {
    Serial.println(F("Failed to write to file"));
  }

  // Close the file
  file.close();
}

// Prints the content of a file to the Serial
void printFile(const char *filename) {
  // Open file for reading
  File file = SD.open(filename);
  if (!file) {
    Serial.println(F("Failed to read file"));
    return;
  }

  // Extract each characters by one by one
  while (file.available()) {
    Serial.print((char)file.read());
  }
  Serial.println();

  // Close the file
  file.close();
}




extern "C" char* sbrk(int incr);

void display_freeram(){
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

int freeRam() {
  char top;
  return &top - reinterpret_cast<char*>(sbrk(0));
}

void pinA1int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[0] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[0] = 1;
}
void pinA2int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[1] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[1] = 1;
}
void pin0int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[2] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[2] = 1;
}
void pin1int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[3] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[3] = 1;
}
void pin3int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[4] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[4] = 1;
}
void pin4int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[5] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[5] = 1;
}
void pin6int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[6] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[6] = 1;
}
void pin7int()
{
  static uint32_t previousmicros = 0;  
  uint32_t thispulsemicros;
  thispulsemicros = micros();
  lastpulseperiod[7] = thispulsemicros - previousmicros;
  previousmicros = thispulsemicros;
  newpulseflag[7] = 1;
}
