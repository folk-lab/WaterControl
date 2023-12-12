/*
Dec 2023, PHAS E-lab
Hennings Water Control PLC
Controls a water pump, monitoring for low-level, overload, and overpressure
Controls a city-water valve, activating on low level, and high temperature
Connects to logging server to log flow meters and temperature sensors
 */

#include <Ethernet.h>
#include <SPI.h>
#include <SD.h>
#include <P1AM.h>
#include <HttpClient.h>
#include <Time.h>
#include <RTCZero.h>
#include <ArduinoRS485.h> // ArduinoModbus depends on the ArduinoRS485 library
#include <ArduinoModbus.h>
#include <ArduinoJson.h>
#include "FlowSensor.h"

#define NUMPULSEINPUTS 8
#define TIMEOUTMICROS 2000000 //If more micros have elapsed since last pulse, set pulse length to default
#define DEFAULTMICROS 999999999
#define NUMFLOWSENSORS 4

#define LEDFLASHMILLIS 1000 //Flashing frequency for LED indicators
#define FLOATDELAYMILLIS 2000 //Delay before acting on float switches
#define TEMPDELAYMILLIS 2000//Delay before acting on tank temperature
#define LOGGERUPDATEMILLIS 5000 //Interval to update logging server values
#define DOSLOT 1 //slot location of 8DO module
#define DISLOT 2 //slot location of 8DI module
#define RTDSLOT 3 //slow location of 4RTD module
#define NUMRTDS 4 //Number of RTDs per module

//Errors ranges for RTD, if reading outside, assume sensor broken
#define RTDMINTEMP -20.0
#define RTDMAXTEMP 50.0

//assign tag names to DI and DO channels

#define DI_PRESSURE_OK 0x01
#define DI_MTR_OL_OK 0x02
#define DI_TANK_ABOVE_LOW 0x04
#define DI_TANK_BELOW_FULL 0x08
#define DI_START 0x10
#define DI_STOP 0x20

#define DO_FILL_V 0x01
#define DO_K_MOTOR 0x02
#define DO_START_LED 0x04
#define DO_STOP_LED 0x08

//Ethernet retry count and timeout in millisec
#define ETH_RETRANSMIT_COUNT 1
#define ETH_RETRANSMIT_TIMEOUT 50

//Modbus holding register addresses for valve-on and valve-off delta settings
#define VALVE_ON_MB_ADDRESS 0
#define VALVE_OFF_DELTA_MB_ADDRESS 2

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
#define DEFAULTVALVEONTEMP 18.0 //default temperature to turn on city water
#define DEFAULTVALVEDELTATEMP 3.0 //temp must be ontemp - delta to turn off
#define DEFAULTVALVE_RTDNUM 0 //index of rtd temp array to use for valve

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
//char ip[]="192.168.1.20";
IPAddress subnet(255, 255, 255, 0); //subnet mask
IPAddress gateway(192, 168, 1, 1); //IP Address of the gateway.
IPAddress dns(8, 8, 8, 8); //IP Address of the DNS server.
IPAddress timeServer(216, 239, 35, 8);  // Google's ntp server address

IPAddress loggingServer;//datalogging server to connect to, defined in config

RTCZero rtc;  //rtc object to get the time from

#define MINUTE 60
#define HOUR 60 * MINUTE
#define DAY 24 * HOUR
const unsigned int ntpSyncTime = DAY; //86400 seconds. Resync every 24 hours.

//If your timezone isn't here, lookup an epoch offset list to find the correct value
const long EDT = -4*HOUR;// epoch offset for EDT timezone
const long EST = -5*HOUR;// epoch offset for EST timezone
const long PDT = -7*HOUR;// epoch offset for PDT timezone
const long PST = -8*HOUR;// epoch offset for PST timezone
long timeZoneOffset = PST;  // timezone to report time from. 


unsigned int localPort = 8888;
const int NTP_PACKET_SIZE = 48; //size of ntp buffer
byte packetBuffer[NTP_PACKET_SIZE]; // buffer to store ntp data

unsigned long ntpLastUpdate = 0;  // var to store time of last sync

typedef struct FlowConfig
{
  char tag[32];
  char location[32];
  char flowunit[16];
  char tempunit[16];
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

typedef struct RTDConfig
{
  char tag[32];
  char location[32];
  char tempunit[16];
}RTDConfig;

// Our configuration structure.
//
// Never use a JsonDocument to store the configuration!
// A JsonDocument is *not* a permanent storage; it's only a temporary storage
// used during the serialization phase. See:
// https://arduinojson.org/v6/faq/why-must-i-create-a-separate-config-object/
struct Config {
  FlowConfig flowconfig[NUMFLOWSENSORS];
  RTDConfig rtdconfig[NUMRTDS];
  uint8_t valvertdnum;
  float valveontemp;
  float valvedeltatemp;
  char serverip[64];
  int serverport;
};

const char *filename = "/config.txt";  // <- SD library uses 8.3 filenames
Config config;                         // <- global configuration object

typedef enum PumpState
{
  STANDBY_OFF,
  FAULT_OFF,
  ON
} PumpState;

PumpState pumpstate = STANDBY_OFF;

typedef enum ValveState
{
  OFF,
  LEVEL_FILLING,
  TEMP_FILLING,
} ValveState;

ValveState valvestate = OFF;
uint8_t lastfaultstate = 0x00;

// Initialize the Ethernet server library
// with the IP address and port you want to use
// (port 80 is default for HTTP):
EthernetServer server(80);
EthernetServer mbServer(502);
EthernetClient client;
HttpClient http=HttpClient(client, loggingServer, config.serverport);
ModbusTCPServer modbusTCPServer;
EthernetUDP Udp;  // udp instance

FlowSensor flowsensor[NUMFLOWSENSORS] = 
{
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 10.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 60.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS),
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 9.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 55.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS),
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 8.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 50.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS),
  FlowSensor(DEFAULTMAXFLOWSCALE, DEFAULTMINFLOWSCALE, 7.0, DEFAULTFLOWFREQ, DEFAULTMAXTEMPSCALE, DEFAULTMINTEMPSCALE, 45.0, DEFAULTTEMPFREQ, DEFAULTTIMEOUTMICROS)
};

//RTD module config: all channels on, DegC, high-side burnout, pt100, 33Hz digital filter
const char P1_04RTD_CONFIG[] = { 0x40, 0x03, 0x60, 0x03, 0x20, 0x01, 0x80, 0x00 };

uint32_t lastMillis, nowMillis, nowMicros = 0;
uint32_t lastLoopTime = 0;
uint32_t longestLoopTime = 0;
uint32_t loopcount = 0;
uint32_t lowlevelmillis, highlevelmillis, tempmillis, loggermillis = 0; //timers for low and high level switches, temperature, logger update
uint8_t faultflag = 0;
uint8_t dibyte = 0;
uint8_t dobyte = 0;
float rtd[NUMRTDS];
float valveontemp = DEFAULTVALVEONTEMP;
float valvedeltatemp = DEFAULTVALVEDELTATEMP;
bool loggingserverconnected = false;

void setup() {

  pinMode(A1, INPUT_PULLUP);
  pinMode(A2, INPUT_PULLUP);  
  pinMode(0, INPUT_PULLUP);
  pinMode(1, INPUT_PULLUP);
  pinMode(3, INPUT_PULLUP);
  pinMode(4, INPUT_PULLUP);
  pinMode(6, INPUT_PULLUP);
  pinMode(7, INPUT_PULLUP);
 
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  /*
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  */ 
  // start the SPI library:
  SPI.begin();

  while (!P1.init())
  { 
    ; //Wait for Modules to Sign on   
  }
  //Configure watchdog to reset after 5000ms timeout
  P1.configWD(5000, TOGGLE);
  P1.startWD();

  P1.configureModule(P1_04RTD_CONFIG, RTDSLOT);//send config to RTD module

  // Initialize SD library
  const int chipSelect = SDCARD_SS_PIN;
  while (!SD.begin(chipSelect)) {
    Serial.println(F("Failed to initialize SD library"));
    delay(1000);
  }
  Serial.println(F("Loading configuration..."));
  loadConfiguration(filename, config);

  // Create configuration file
  //Serial.println(F("Saving configuration..."));
  //saveConfiguration(filename, config);
  display_freeram();

  // start the Ethernet connection
  Ethernet.begin(mac, ip, dns, gateway, subnet);

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
  Ethernet.setRetransmissionCount(ETH_RETRANSMIT_COUNT);
  Ethernet.setRetransmissionTimeout(ETH_RETRANSMIT_TIMEOUT);
  if (loggingServer.fromString(config.serverip))
  { // try to parse into the IPAddress
    Serial.println(loggingServer); // print the parsed IPAddress 
  }
  else
  {
    Serial.println("UnParsable IP");
  }
  client.setConnectionTimeout(ETH_RETRANSMIT_TIMEOUT);
  if (client.connect(loggingServer, config.serverport))
  {
    loggingserverconnected = true;
    Serial.println("Connected to server");
  }
  else
  {
    loggingserverconnected = false;
    Serial.println("Could not connect to server...");
  }
  if (client.connected())
  {
    Serial.println("Connected. Stopping now");
  }
  http=HttpClient(client, loggingServer, config.serverport);
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
  
  rtc.begin();  // start real-time-clock
  getTimeAndDate();   // initial sync from ntp server
  printTime();

  //reinitialize flowsensors with loaded config values
  for(int i = 0; i < NUMFLOWSENSORS; i++)
  {
    flowsensor[i].reinit(config.flowconfig[i].maxflowscale,config.flowconfig[i].minflowscale, config.flowconfig[i].flowthresh, config.flowconfig[i].flowfreq,
    config.flowconfig[i].maxtempscale, config.flowconfig[i].mintempscale, config.flowconfig[i].tempthresh, config.flowconfig[i].tempfreq, config.flowconfig[i].timeoutmicros);
  }

  //Just using 4 registers for now, to save the valve on temp at registers 0-1 and valve hysteresis at registers 2-3,
  //may change to something http based if necessary

  modbusTCPServer.configureHoldingRegisters(0x00, 4);
  
  modbuswritefloat(VALVE_ON_MB_ADDRESS, config.valveontemp);
  modbuswritefloat(VALVE_OFF_DELTA_MB_ADDRESS, config.valvedeltatemp);

}



void loop() {

  //Reset watchdog timer
  P1.petWD();
  updateio();
  nowMillis = millis();
  nowMicros = micros();
  lastLoopTime = nowMillis - lastMillis;
  lastMillis = nowMillis;
  faultflag = 0;
  //process flow sensors
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
  //process pump
  switch(pumpstate)
  {
    case STANDBY_OFF:
    {
      dobyte &= ~DO_K_MOTOR; //clear motor contactor bit
      dobyte |= DO_STOP_LED;//set red LED
      flashstartled();
      if(dibyte & DI_START)//if start button pushed
      {
        pumpstate = ON;
      }      
      break;
    }
    case FAULT_OFF:
    {
      dobyte &= ~DO_K_MOTOR; //clear motor contactor bit
      dobyte &= ~DO_START_LED; //clear green button LED
      flashstopled();
      if(!(dibyte & DI_STOP)) //stop button acknowledges fault
      {
        pumpstate = STANDBY_OFF;
      }
      break;
    }
    case ON:
    {
      dobyte &= ~DO_STOP_LED;//clear red LED
      dobyte |= DO_START_LED;//set green LED
      if(dibyte & DI_TANK_ABOVE_LOW)//tank level above low, reset low level timer
      {
        lowlevelmillis = nowMillis;
      }
      if(!(dibyte & DI_STOP)) //stop button
      {
        pumpstate = STANDBY_OFF;
      }      
      if(!(dibyte & DI_MTR_OL_OK) || !(dibyte & DI_PRESSURE_OK)) //motor overload or pump pressure signals missing should fault immediately 
      {
        lastfaultstate = dibyte;
        pumpstate = FAULT_OFF;
      }
      if(!(dibyte & DI_TANK_ABOVE_LOW)) //low level should fault if persists longer than delay
      {
        if((nowMillis - lowlevelmillis) >= FLOATDELAYMILLIS)
        {
          pumpstate = FAULT_OFF;
          //last fault = 
        }        
      }
      if(pumpstate == ON)//if state is still on we can start the motor
      {
        dobyte |= DO_K_MOTOR; //set motor contactor bit
      }
      else//this is redundant, but response will be faster if we clear now
      {
        dobyte &= ~DO_K_MOTOR;//clear motor contactor bit
        updateio();
      }
      break;
    }
  }
  //process water valve
  switch(valvestate)
  {
    case OFF:
    {
      dobyte &= ~DO_FILL_V;
      if(!(dibyte & DI_TANK_BELOW_FULL))//tank level above full, reset level timer
      {
        highlevelmillis = nowMillis;
      }
      else//tank below full, check for delay time
      {
        if(((nowMillis - highlevelmillis) >= FLOATDELAYMILLIS) && (pumpstate == ON))
        {
          valvestate = LEVEL_FILLING;
          //send event
        }
      }
      //above temp or broken, wait for timer
      if((rtd[config.valvertdnum] >= (config.valveontemp - config.valvedeltatemp)) || (rtd[config.valvertdnum] < RTDMINTEMP) || (rtd[config.valvertdnum] > RTDMAXTEMP))
      {
        if(((nowMillis - tempmillis) >= TEMPDELAYMILLIS) && (pumpstate == ON))
        {
          valvestate = TEMP_FILLING;
        }
      }
      else//below temp, reset temp timer
      {
        tempmillis = nowMillis;
      }      
      break;
    }
    case LEVEL_FILLING:
    {
      if(dibyte & DI_TANK_BELOW_FULL)//tank below full, reset level timer
      {
        highlevelmillis = nowMillis;
      }
      else
      {
        if((nowMillis - highlevelmillis) >= FLOATDELAYMILLIS)//if above full for delay period
        {
          valvestate = OFF;
        }
      }
      if(pumpstate != ON)
      {
        valvestate = OFF;
      }
      if(valvestate == LEVEL_FILLING)
      {
        dobyte |= DO_FILL_V;
      }      
      break;
    }
    case TEMP_FILLING:
    {
      if((rtd[config.valvertdnum] >= (config.valveontemp - config.valvedeltatemp)) || (rtd[config.valvertdnum] < RTDMINTEMP) || (rtd[config.valvertdnum] > RTDMAXTEMP))//still above temp or broken, reset delay timer
      { 
        tempmillis = nowMillis;
      }
      else
      {
        if(nowMillis - tempmillis >= TEMPDELAYMILLIS)//if below temp for delay
        {
          valvestate = OFF;
          //send event
        }
      }
      if(pumpstate != ON)
      {
        valvestate = OFF;
      }
      if(valvestate == TEMP_FILLING)
      {
         dobyte |= DO_FILL_V;
      }
      break;
    }
  }
  //update logging server if time expired, also check NTP sync
  if(nowMillis - loggermillis >= LOGGERUPDATEMILLIS)
  {
    loggermillis = nowMillis;
    if(Ethernet.linkStatus() == LinkON)
    {
      //checkForNTPResync();
      //char time[64];
      //sprintf(time, "20%d-%02d-%02dT%02d:%02d:%02dZ", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      //Serial.println(time);
      if (client.connect(loggingServer, config.serverport))
      {
        loggingserverconnected = true;
      // Serial.println("Connected to server");
      }
      else
      {
        loggingserverconnected = false;
        Serial.println("Could not connect to server...");
      }
    
    }
    else
    {
      Serial.println("No Ethernet Link");
    }
    //If server connected, proceed
    if(loggingserverconnected)
    {
      checkForNTPResync();
      char nowtime[64];
      sprintf(nowtime, "20%d-%02d-%02dT%02d:%02d:%02dZ", rtc.getYear(), rtc.getMonth(), rtc.getDay(), rtc.getHours(), rtc.getMinutes(), rtc.getSeconds());
      Serial.println(nowtime);
      for(int i = 0; i < NUMFLOWSENSORS; i++)
      {
        updatelogger(config.flowconfig[i].location, config.flowconfig[i].tag, "Flow", config.flowconfig[i].flowunit, flowsensor[i].getflowscaled(), nowtime);
        updatelogger(config.flowconfig[i].location, config.flowconfig[i].tag, "Temperature", config.flowconfig[i].tempunit, flowsensor[i].gettempscaled(), nowtime);
      }
      for(int i = 0; i < NUMRTDS; i++)
      {
        updatelogger(config.rtdconfig[i].location, config.rtdconfig[i].tag, "Temperature", config.rtdconfig[i].tempunit, rtd[i], nowtime);
      }
    }    
  }  

  loopcount++;
  if(loopcount > 5000)
  {
    longestLoopTime = 0;
    loopcount = 0;
  }

  if(lastLoopTime > longestLoopTime)  
  {
    longestLoopTime = lastLoopTime;
  }

  // listen for incoming Ethernet connections:
  listenForEthernetClients();
  handlemodbus();
}

/*
String buildJSON(String LOCATION, String TAG, String Data, String Unit, String Time)
{
  return "{  \"log_session\": {\n    \"location\": \"" + LOCATION + "\",\n    \"tag\": \"" + TAG + "\"\n  },\n  \"record\": {\n    \"temperature\": {\n      \"unit\": \"" + Unit + "\",\n      \"value\": \"" + Data + "\"\n    },\n    \"time\": \"" + Time + "\"\n}}";
}
*/

void updatelogger(char * LOCATION, char * TAG, const char * VARIABLE, char * UNIT, float DATA, String time)
{
String output;
StaticJsonDocument<192> doc;

JsonObject log_session = doc.createNestedObject("log_session");
log_session["location"] = LOCATION;
log_session["tag"] = TAG;

JsonObject record = doc.createNestedObject("record");

JsonObject record_variable = record.createNestedObject(VARIABLE);
record_variable["unit"] = UNIT;
record_variable["value"] = DATA;
record["time"] = time;

serializeJsonPretty(doc, output);
PUTrequest(output);
}

void PUTrequest(String json) {
  http.beginRequest();
  //http.put(loggingServer, "/api/v1/log-data");
  http.put("/api/v1/log-data");
  http.sendHeader("accept: application/json");
  http.sendHeader("Content-Type: application/json");
  http.sendHeader("lcmi-auth-token: swagger");
  http.sendHeader("Content-Length", json.length());
  http.beginBody();
  http.print(json);
  http.endRequest();
}

/////////////////////////////////////////////////////////////////
/////////////////// GET REQUEST /////////////////////////////////
/////////////////////////////////////////////////////////////////
void GETrequest() {
  http.beginRequest();
  http.get("/api/v1/get-all-log-sessions");
  http.sendHeader("accept: application/json");
  http.sendHeader("lcmi-auth-token: swagger");
  http.endRequest();
}



void updateio(void)
{
  char  littleEndianTemp[16];			//Array to store unformatted little endian temperature Bytes
  char  bigEndianTemp[16];		    //Array to store unformatted big endian temperature Bytes  
  
  dibyte = P1.readDiscrete(DISLOT);
  P1.writeDiscrete(dobyte, DOSLOT);
  P1.readBlockData(littleEndianTemp, (NUMRTDS * 4), 0, ANALOG_IN_BLOCK);
  //Reverse byte order to correct for endianess
  for(int i = 0; i < NUMRTDS; i++){
    bigEndianTemp[4*i + 3] = littleEndianTemp[4*i + 0];   
    bigEndianTemp[4*i + 2] = littleEndianTemp[4*i + 1];  
    bigEndianTemp[4*i + 1] = littleEndianTemp[4*i + 2];  
    bigEndianTemp[4*i + 0] = littleEndianTemp[4*i + 3];  
  }
  memcpy(rtd, bigEndianTemp, (NUMRTDS *4)); //Copy 16 bytes into our float array. Floats are 4 bytes each.
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
      float v_on, v_off;
      v_on = modbusreadfloat(VALVE_ON_MB_ADDRESS);
      v_off = modbusreadfloat(VALVE_OFF_DELTA_MB_ADDRESS);      
      if((v_on != config.valveontemp) || (v_off != config.valvedeltatemp))
      {
        config.valveontemp = v_on;
        config.valvedeltatemp = v_off;
        Serial.print("New valve-on temp: ");
        Serial.println(config.valveontemp);
        Serial.print("New valve-off temp: ");
        Serial.println(config.valvedeltatemp);
        Serial.println(F("Saving configuration..."));
        saveConfiguration(filename, config);
      }
    }   
  } 
  else
  {
    accepted = false;
  }
}

//writes a float to the Modbus holding register address specified, and address + 1, float is 4 bytes, modbus register 2 bytes
void modbuswritefloat(int address, float value)
{
  // float voltage value cast to 32bit int and split between 2x 16bit ints
  uint32_t *dualValue1 = (uint32_t *)&value;

  uint16_t register1 = *dualValue1 >> 16;  
  uint16_t register2 = *dualValue1 & 0xFFFF;

  modbusTCPServer.holdingRegisterWrite(address, register1);
  modbusTCPServer.holdingRegisterWrite((address + 1), register2);
}

//reads a float from the Modbus holding register address specified, and address + 1,
float modbusreadfloat(int address)
{
  uint16_t register1, register2;
  uint32_t dualValue1;
  register1 = modbusTCPServer.holdingRegisterRead(address);
  register2 = modbusTCPServer.holdingRegisterRead(address + 1);
  dualValue1 = register1;
  dualValue1 = dualValue1 << 16;
  dualValue1 |= register2;
  float *returnfloat = (float *)&dualValue1;
  return *returnfloat;
}


void listenForEthernetClients() {
  // listen for incoming clients
  EthernetClient sclient = server.available();
  String readString; 
  if (sclient) {
    //Serial.println("Got a client");
    // an HTTP request ends with a blank line
    bool currentLineIsBlank = true;
    while (sclient.connected()) {
      if (sclient.available()) {
        char c = sclient.read();
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
          sclient.println("HTTP/1.1 200 OK");
          sclient.println("Content-Type: text/html");
          sclient.println("Connection: close");  // the connection will be closed after completion of the response
          if (readString.indexOf("?Autorefresh") >0 )
          {
            sclient.println("Refresh: 1");  // refresh the page automatically every 5 sec
          }
          sclient.println();
          sclient.println("<!DOCTYPE HTML>");
          sclient.println("<html>");          
          sclient.println();
          sclient.println("<a href=\"/?Autorefresh\"\">AutoRefresh</a>");
          sclient.println("<br />");
          sclient.println("<a href=\"/\"\">StopAutoRefresh</a>");
          sclient.println("<br />");
          sclient.println("<a href=\"/?buttonResetFault\"\">Reset Faults</a>");
          sclient.println("<br />");

          for(int i = 0; i < NUMFLOWSENSORS; i++)
          {
            sclient.print("Flow ");
            sclient.print(i);
            sclient.print(" Pulse period: ");
            sclient.print(flowsensor[i].getflowmicros());
            sclient.print("us");
            sclient.println("<br />");
            sclient.print("Flow ");
            sclient.print(i);
            sclient.print(" freq: ");
            sclient.print(flowsensor[i].getflowfreq());
            sclient.print("Hz");
            sclient.println("<br />");
            sclient.print("Flow ");
            sclient.print(i);
            sclient.print(" Scaled: ");
            sclient.print(flowsensor[i].getflowscaled());
            sclient.print("LPM");
            sclient.println("<br />");
            sclient.print("Flow ");
            sclient.print(i);
            sclient.print(" Fault: ");
            sclient.print(flowsensor[i].getflowfault());
            sclient.println("<br />");            
            sclient.print("Temp ");
            sclient.print(i);
            sclient.print(" Pulse period: ");
            sclient.print(flowsensor[i].gettempmicros());
            sclient.print("us");
            sclient.println("<br />");
            sclient.print("Temp ");
            sclient.print(i);
            sclient.print(" freq: ");
            sclient.print(flowsensor[i].gettempfreq());
            sclient.print("Hz");
            sclient.println("<br />");
            sclient.print("Temp ");
            sclient.print(i);
            sclient.print(" scaled: ");
            sclient.print(flowsensor[i].gettempscaled());
            sclient.print("DegC");
            sclient.println("<br />");
            sclient.print("Temp ");
            sclient.print(i);
            sclient.print(" Fault: ");
            sclient.print(flowsensor[i].gettempfault());
            sclient.println("<br />");
          }
          sclient.print("DI Byte: ");
          sclient.print(dibyte, HEX);
          sclient.println("<br />");
          sclient.print("DO Byte: ");
          sclient.print(dobyte, HEX);
          sclient.println("<br />");

          for(int i = 0; i < NUMRTDS; i++)
          {
            sclient.print("RTD Temp ");
            sclient.print(i);
            sclient.print(": ");
            sclient.print(rtd[i]);
            sclient.print("DegC");
            sclient.println("<br />");
          }
          sclient.print("Millis: ");
          sclient.print(millis());
          sclient.println("<br />");
          sclient.print("LoopTime: ");
          sclient.print(lastLoopTime);
          sclient.println("<br />");
          sclient.print("longestLoopTime: ");
          sclient.print(longestLoopTime);
          sclient.println("<br />");
          sclient.print("loopcount: ");
          sclient.print(loopcount);
          sclient.println("<br />");
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
    sclient.stop();
    
  
  }
}
// Loads the configuration from a file
void loadConfiguration(const char *filename, Config &config) {
  // Open file for reading
  File file = SD.open(filename);

  // Allocate a temporary JsonDocument
  DynamicJsonDocument doc(2048);

  // Deserialize the JSON document
  DeserializationError error = deserializeJson(doc, file);
  if (error)
    Serial.println(F("Failed to read file, using default configuration"));
int i = 0;
for (JsonObject flowsensor : doc["flowsensors"].as<JsonArray>()) {

  strlcpy(config.flowconfig[i].tag, flowsensor["tag"] | "Undefined", sizeof(config.flowconfig[i].tag));
  strlcpy(config.flowconfig[i].location, flowsensor["location"] | "Undefined", sizeof(config.flowconfig[i].location));
  strlcpy(config.flowconfig[i].flowunit, flowsensor["flowunit"] | "lpm", sizeof(config.flowconfig[i].flowunit)); // "lpm", "lpm", "lpm", "lpm"
  strlcpy(config.flowconfig[i].tempunit, flowsensor["tempunit"] | "degC", sizeof(config.flowconfig[i].tempunit)); // "degC", "degC", "degC", "degC"
  config.flowconfig[i].maxflowscale = flowsensor["maxflowscale"] | DEFAULTMAXFLOWSCALE; // 20, 20, 20, 20
  config.flowconfig[i].minflowscale = flowsensor["minflowscale"] | DEFAULTMINFLOWSCALE; // 0, 0, 0, 0
  config.flowconfig[i].flowthresh = flowsensor["flowthresh"] | 1.0; // 9, 9, 9, 9
  config.flowconfig[i].flowfreq = flowsensor["flowfreq"] | DEFAULTFLOWFREQ; // 100, 100, 100, 100
  config.flowconfig[i].maxtempscale = flowsensor["maxtempscale"] | DEFAULTMAXTEMPSCALE; // 65.56, 65.56, 65.56, 65.56
  config.flowconfig[i].mintempscale = flowsensor["mintempscale"] | DEFAULTMINTEMPSCALE; // -10, -10, -10, -10
  config.flowconfig[i].tempfreq = flowsensor["tempfreq"] | DEFAULTTEMPFREQ; // 100, 100, 100, 100
  config.flowconfig[i].timeoutmicros = flowsensor["timeoutmicros"] | DEFAULTTIMEOUTMICROS; // 3000000, 3000000, 3000000, 3000000
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
i = 0;
for (JsonObject tempsensor : doc["tempsensors"].as<JsonArray>()) 
{
  strlcpy(config.rtdconfig[i].tag, tempsensor["tag"] | "Undefined", sizeof(config.rtdconfig[i].tag)); // "Tempsensor1", "Tempsensor2", "Tempsensor3", ...
  strlcpy(config.rtdconfig[i].location, tempsensor["location"] | "Undefined", sizeof(config.rtdconfig[i].location)); // "Compressor Room", "Compressor Room", ...
  strlcpy(config.rtdconfig[i].tempunit, tempsensor["tempunit"] | "degC", sizeof(config.rtdconfig[i].tempunit)); // "degC", "degC", "degC", "degC"
  Serial.println(config.rtdconfig[i].tag);
  Serial.println(config.rtdconfig[i].location);  
  Serial.println(config.rtdconfig[i].tempunit);
  i++;
  if(i >= NUMRTDS)
  {
  break;
  }
}

config.valvertdnum = doc["valvertdnum"] | 0; // 0
config.valveontemp = doc["valveontemp"] | 18.0; // 18
config.valvedeltatemp = doc["valvedeltatemp"] | 3.0; // 3
strlcpy(config.serverip, doc["serverip"] |"192.168.1.30", sizeof(config.serverip)); // "loggingserver"
config.serverport = doc["serverport"] | 80; // 21
Serial.println(config.valvertdnum);
Serial.println(config.valveontemp);  
Serial.println(config.valvedeltatemp);
Serial.println(config.serverip);
Serial.println(config.serverport);


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
DynamicJsonDocument doc(2048);

JsonArray flowsensors = doc.createNestedArray("flowsensors");

for(int i = 0; i < NUMFLOWSENSORS; i++)
{
  JsonObject flowsensors_i = flowsensors.createNestedObject();
  flowsensors_i["tag"] = config.flowconfig[i].tag;
  flowsensors_i["location"] = config.flowconfig[i].location;
  flowsensors_i["flowunit"] = config.flowconfig[i].flowunit;
  flowsensors_i["tempunit"] = config.flowconfig[i].tempunit;
  flowsensors_i["maxflowscale"] = config.flowconfig[i].maxflowscale;
  flowsensors_i["minflowscale"] = config.flowconfig[i].minflowscale;
  flowsensors_i["flowthresh"] = config.flowconfig[i].flowthresh;
  flowsensors_i["flowfreq"] = config.flowconfig[i].flowfreq;
  flowsensors_i["maxtempscale"] = config.flowconfig[i].maxtempscale;
  flowsensors_i["mintempscale"] = config.flowconfig[i].mintempscale;
  flowsensors_i["tempfreq"] = config.flowconfig[i].tempfreq;
  flowsensors_i["timeoutmicros"] = config.flowconfig[i].timeoutmicros;
}

JsonArray tempsensors = doc.createNestedArray("tempsensors");

for(int i = 0; i < NUMRTDS; i++)
{
  JsonObject tempsensors_i = tempsensors.createNestedObject();
  tempsensors_i["tag"] = config.rtdconfig[i].tag;
  tempsensors_i["location"] = config.rtdconfig[i].location;
  tempsensors_i["tempunit"] = config.rtdconfig[i].tempunit;
}

doc["valvertdnum"] = config.valvertdnum;
doc["valveontemp"] = config.valveontemp;
doc["valvedeltatemp"] = config.valvedeltatemp;
doc["serverip"] = config.serverip;
doc["serverport"] = config.serverport;

  // Serialize JSON to file
  if (serializeJsonPretty(doc, file) == 0) {
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

void flashstopled()
{
  static uint32_t lastledtimer = 0;
  if((nowMillis - lastledtimer) >= (LEDFLASHMILLIS /2))
  {
    lastledtimer = nowMillis;
    if(dobyte & DO_STOP_LED)
    {
      dobyte &= ~DO_STOP_LED;
    }
    else
    {
      dobyte |= DO_STOP_LED;
    }
  }
}

void flashstartled()
{
  static uint32_t lastledtimer = 0;
  if((nowMillis - lastledtimer) >= (LEDFLASHMILLIS /2))
  {
    lastledtimer = nowMillis;
    if(dobyte & DO_START_LED)
    {
      dobyte &= ~DO_START_LED;
    }
    else
    {
      dobyte |= DO_START_LED;
    }
  }
}

void checkForNTPResync()
{
  unsigned long currentEpoch = rtc.getEpoch();  // current total RTC seconds
  if (currentEpoch - ntpLastUpdate > ntpSyncTime) {   // resync time after  ntpSyncTime interval. We have this for every day above
    if(getTimeAndDate()) {  // try to resync time from ntp server
      Serial.println("Resync successful.");
    }
    else {
      Serial.println("Resync failed.");
    }
  }  
}

void printTime() {  // print time and date from rtc
  Serial.print(rtc.getHours());
  printDigits(rtc.getMinutes());
  printDigits(rtc.getSeconds());
  Serial.println();
  Serial.print(rtc.getMonth());
  Serial.print("/");
  Serial.print(rtc.getDay());
  Serial.print("/");
  Serial.println(rtc.getYear());
  Serial.println();
}

void printDigits(int digits) {  // print preceding ':' and '0' for time
  Serial.print(':');
  if (digits < 10)
    Serial.print('0');
  Serial.print(digits);
}


// send an NTP request to the time server at the given address
void sendNTPpacket(IPAddress& address) {
  // set all bytes in the buffer to 0
  memset(packetBuffer, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  packetBuffer[0] = 0b11100011;   // LI, Version, Mode
  packetBuffer[1] = 0;     // Stratum, or type of clock
  packetBuffer[2] = 6;     // Polling Interval
  packetBuffer[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  packetBuffer[12]  = 49;
  packetBuffer[13]  = 0x4E;
  packetBuffer[14]  = 49;
  packetBuffer[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  Udp.beginPacket(address, 123); //NTP requests are to port 123
  Udp.write(packetBuffer, NTP_PACKET_SIZE);
  Udp.endPacket();
}
int getTimeAndDate() {  // sync time on rtc from ntp server
  int flag = 0;
  Udp.begin(localPort);
  sendNTPpacket(timeServer); // send packet to ntp server
  delay(250);
  if (Udp.parsePacket()) {  // read response from ntp server
    Udp.read(packetBuffer, NTP_PACKET_SIZE); // read the packet into the buffer
    unsigned long highWord, lowWord, epoch;
    highWord = word(packetBuffer[40], packetBuffer[41]);
    lowWord = word(packetBuffer[42], packetBuffer[43]);
    epoch = highWord << 16 | lowWord;
    epoch = epoch - 2208988800 + timeZoneOffset;
    flag = 1;
    rtc.setEpoch(epoch);  // set rtc to up-to-date epoch
    ntpLastUpdate = epoch;  // set last time time was synced
  }
  return flag;  // return true on successful update
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
