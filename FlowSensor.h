/*
FlowSensor Class
*/
#ifndef FLOWSENSOR_H
#define FLOWSENSOR_H
//#define DEBUGPRINT
#define NUMFLOWAVERAGES 4
#define NUMTEMPAVERAGES 4
#define TIMEOUTFLOWMICROS 999999999
#define TIMEOUTTEMPMICROS 999
#define MICROSECPERSEC 1000000.0
typedef enum Fault_state{OK = 0, FAULT = 1} Fault_state;

class FlowSensor{
  public:
  FlowSensor(float maxflow, float minflow, float flowthresh, float flowfreq, float maxtemp, float mintemp, float tempthresh, float tempfreq, uint32_t timeoutmicros);
  void reinit(float maxflow, float minflow, float flowthresh, float flowfreq, float maxtemp, float mintemp, float tempthresh, float tempfreq, uint32_t timeoutmicros);
  void process(void);
  void updateflow(uint32_t lastpulsemicros);
  void updatetemp(uint32_t lastpulsemicros);
  uint32_t getflowmicros(void);
  uint32_t gettempmicros(void);
  float getflowfreq(void);
  float gettempfreq(void);
  float getflowscaled(void);
  float gettempscaled(void);
  Fault_state getflowfault(void);
  Fault_state gettempfault(void);
  void resetfault(void);
  uint32_t timeoutus;
  float fmax, fmin, fthresh, ffreq, tmax, tmin, tthresh, tfreq;
  
  Fault_state flowfault_state;
  Fault_state tempfault_state;
  private:
  uint32_t flowmicrosarray[NUMFLOWAVERAGES];
  uint32_t tempmicrosarray[NUMTEMPAVERAGES];
  uint32_t flowavgmicros, tempavgmicros, lastflowmicros, lasttempmicros, flowfaultmicros, tempfaultmicros;
  uint16_t flowavgindex, tempavgindex;
  uint8_t flowflag, tempflag;
  float flowavgfreq, tempavgfreq, flowscaled, tempscaled;
	
};
#endif
