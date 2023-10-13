#include <stdlib.h>
#include <SPI.h>
#include <math.h>
#include <string.h>
#include "FlowSensor.h"

FlowSensor::FlowSensor(float maxflow, float minflow, float flowthresh, float flowfreq, float maxtemp, float mintemp, float tempthresh, float tempfreq, uint32_t timeoutmicros)
{
  fmax = maxflow;//max flow for scaling frequency to flow
  fmin = minflow;//min flow for scaling frequency to flow
  fthresh = flowthresh;//min flow threshold for fault
  ffreq = flowfreq;//max frequency of flow output
  tmax = maxtemp;//max temp for scaling frequency to temp
  tmin = mintemp;//min temp for scaling frequency to temp
  tthresh = tempthresh;//max temp threshold for fault
  tfreq = tempfreq;//max frequency for temp output
  timeoutus = timeoutmicros;//if flow is below or temp is above for specified microseconds, set fault
  bzero(flowmicrosarray, sizeof(flowmicrosarray));
  bzero(tempmicrosarray, sizeof(tempmicrosarray));
  flowavgmicros = 0;
  tempavgmicros = 0;
  lastflowmicros = 0;
  lasttempmicros = 0;
  flowavgindex = 0;
  tempavgindex = 0;
  //waterfaultmicros = 0;
  tempfaultmicros = 0;
  flowfreq = 0.0; 
  tempfreq = 0.0;
  flowscaled = 0.0;
  tempscaled = 0.0;
  flowfault_state = OK;
  tempfault_state = OK;
  flowfaultmicros = 0;
  tempfaultmicros = 0;
  flowflag = 0;
  tempflag = 0;
 
}

void FlowSensor::reinit(float maxflow, float minflow, float flowthresh, float flowfreq, float maxtemp, float mintemp, float tempthresh, float tempfreq, uint32_t timeoutmicros)
{
  fmax = maxflow;
  fmin = minflow;
  fthresh = flowthresh;
  ffreq = flowfreq;
  tmax = maxtemp;
  tmin = mintemp;
  tthresh = tempthresh;
  tfreq = tempfreq;
  timeoutus = timeoutmicros;  
}

/*
process() should be called every main loop, checks for timeouts or value out of range
scales the lastest pulse period into the correct units
*/

void FlowSensor::process(void)
{
	uint32_t nowmicros;
  nowmicros = micros();
  //check for no pulses coming from flow meters
	if(((nowmicros - timeoutus) > lastflowmicros) && (nowmicros > timeoutus))
	{
		flowavgmicros = TIMEOUTFLOWMICROS;
	}
	if(((nowmicros - timeoutus) > lasttempmicros) && (nowmicros > timeoutus))
	{
		tempavgmicros = TIMEOUTTEMPMICROS;
	}	
	
	flowavgfreq = MICROSECPERSEC / flowavgmicros;
	tempavgfreq = MICROSECPERSEC / tempavgmicros;
	flowscaled = ((flowavgfreq / ffreq) * (fmax - fmin)) + fmin;
	tempscaled = ((tempavgfreq / tfreq) * (tmax - tmin)) + tmin;
	//if flow less than threshold, start a timer and fault if time exceeded
  if(flowscaled < fthresh)
	{
		if(flowflag == 0)
    {
      //Serial.print("Low Flow detected: Thresh");
      //Serial.println(fthresh);
      flowfaultmicros = nowmicros;
      flowflag = 1;
    }
    if((nowmicros - flowfaultmicros) > timeoutus)
    {
      flowfault_state = FAULT;
    }    
	}
  //reset timer if flow goes back in range
  else
  {
    flowflag = 0;
  }
  //if temp less than threshold, start a timer and fault if time exceeded
  if(tempscaled > tthresh)
	{
		if(tempflag == 0)
    {
      tempfaultmicros = nowmicros;
      tempflag = 1;
    }
    if((nowmicros - tempfaultmicros) > timeoutus)
    {
      tempfault_state = FAULT;
    }    
	}
  //reset timer if flow goes back in range
  else
  {
    tempflag = 0;
  }			    
}

/*
updateflow() should be called for each new detected pulse, calculates a running average of the last NUMFLOWAVERAGES
pulseperiods to filter out noise. Resets the timeout counter
*/

void FlowSensor::updateflow(uint32_t lastpulsemicros)
{
  uint16_t i = 0;
  uint32_t avgsum = 0;
  lastflowmicros = micros();
  flowmicrosarray[flowavgindex] = lastpulsemicros;
  flowavgindex++;
  if(flowavgindex >= NUMFLOWAVERAGES)
  {
    flowavgindex = 0;
  }
  for(i = 0; i < NUMFLOWAVERAGES; i++)
  {
    avgsum += flowmicrosarray[i];
  }
  flowavgmicros = avgsum / NUMFLOWAVERAGES;
}

/*
updatetemp() should be called for each new detected pulse, calculated a running average of the last NUMTEMPAVERAGES
pulseperiods to filter out noise. Resets the timeout counter
*/

void FlowSensor::updatetemp(uint32_t lastpulsemicros)
{
  uint16_t i = 0;
  uint32_t avgsum = 0;
  lasttempmicros = micros();
  tempmicrosarray[tempavgindex] = lastpulsemicros;
  tempavgindex++;
  if(tempavgindex >= NUMTEMPAVERAGES)
  {
    tempavgindex = 0;
  }
  for(i = 0; i < NUMTEMPAVERAGES; i++)
  {
    avgsum += tempmicrosarray[i];
  }
  tempavgmicros = avgsum / NUMTEMPAVERAGES;
}

uint32_t FlowSensor::getflowmicros(void)
{
  return flowavgmicros;
}

uint32_t FlowSensor::gettempmicros(void)
{
  return tempavgmicros;
}

float FlowSensor::getflowfreq(void)
{
  return flowavgfreq;
}

float FlowSensor::gettempfreq(void)
{
  return tempavgfreq;
}

float FlowSensor::getflowscaled(void)
{
  return flowscaled;
}

float FlowSensor::gettempscaled(void)
{
  return tempscaled;
}
Fault_state FlowSensor::getflowfault(void)
{
  return flowfault_state;
}

Fault_state FlowSensor::gettempfault(void)
{
  return tempfault_state;
}
void FlowSensor::resetfault(void)
{
	uint32_t nowmicros;
  nowmicros = micros();
  flowfaultmicros = nowmicros;
  tempfaultmicros = nowmicros;
  flowfault_state = OK;
	tempfault_state = OK;

}