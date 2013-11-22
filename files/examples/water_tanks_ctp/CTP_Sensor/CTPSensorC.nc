/*
 * Copyright (c) 2013, KTH Royal Institute of Technology
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 *  - Redistributions of source code must retain the above copyright notice, this list
 * 	  of conditions and the following disclaimer.
 *
 * 	- Redistributions in binary form must reproduce the above copyright notice, this
 *    list of conditions and the following disclaimer in the documentation and/or other
 *	  materials provided with the distribution.
 *
 * 	- Neither the name of the KTH Royal Institute of Technology nor the names of its
 *    contributors may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY
 * OF SUCH DAMAGE.
 *
 */
/*
 * @author Behdad Aminian <behdad@kth.se>
 *
*/
#include <Timer.h>
#include "app_parameters.h"


module CTPSensorC {
  uses interface Boot;
  uses interface SplitControl as RadioControl;
  uses interface StdControl as RoutingControl;
  uses interface Send;
  uses interface Leds;
 
  uses interface RootControl;
  uses interface Receive;
  
  //ADC  
  uses interface Resource;
  uses interface Msp430Adc12MultiChannel as MultiChannel;
  provides interface AdcConfigure<const msp430adc12_channel_config_t*>;
  
  //Sensor Code
  uses interface Timer<TMilli> as TimerSensorBoot;
  uses interface Timer<TMilli> as TimerSend;
  
    
}
implementation {
//EasyCollection
  message_t packet;
  bool sendBusy = FALSE;


//Sensor
const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};
	
	uint16_t SampleRate = SAMPLING_PERIOD;
	uint16_t buffer[3];
	SensorValues sensorValues;
	
// Prototype functions

	task void getData();
	task void sendMessage();
	
	uint16_t nSend=1; //Number of sent packets
  event void Boot.booted() {
  call TimerSensorBoot.startOneShot(1000);
  }
  
  event void TimerSensorBoot.fired() {
		call RadioControl.start();
	}
  
  event void RadioControl.startDone(error_t err) {
		if (err == SUCCESS) {
		
		call RoutingControl.start();
			//if (TOS_NODE_ID==COORDINATOR_ADDRESS)
			//call RootControl.setRoot();
		
		call Resource.request();
		}
		else {
			call RoutingControl.start();
		}
	}
	event void RadioControl.stopDone(error_t err) {
	}
	 /*********************************************************************
	 * Multi Channel functions
	 *********************************************************************/
	 
  event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) {
				call Leds.led0On();
			}
		}
		// Start a periodic timer to read the ADC
		call TimerSend.startPeriodic(SampleRate);
	}
	
	event void TimerSend.fired() {
		post getData();
	}
	
	task void getData()
	{
		// Read ADC values
		call MultiChannel.getData();
	}
   
  async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		// Copy sensor readings
		sensorValues.tankLevel[0] = buf[1];
		sensorValues.tankLevel[1] = buf[2];
		
		if (!sendBusy)
		post sendMessage();
	}
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/
	 /*********************************************************************
	 * Send Message
	 *********************************************************************/
	 
task void sendMessage(){
 SensorValues* msg =
      (SensorValues*)call Send.getPayload(&packet, sizeof(SensorValues));
atomic {
      msg->tankLevel[0]=sensorValues.tankLevel[0];
      msg->tankLevel[1]=sensorValues.tankLevel[1];
	  msg->sensor_ID=TOS_NODE_ID;
	  msg->pkt_ID=nSend;
   }
    if (call Send.send(&packet, sizeof(SensorValues)) != SUCCESS) 
      call Leds.led0On();
    else 
      sendBusy = TRUE;
}
  
  event void Send.sendDone(message_t* m, error_t err) {
    if (err != SUCCESS) 
      call Leds.led0On();
	
    sendBusy = FALSE;

	nSend++;
  }
  	/*********************************************************************
	 * END Send Message
   	*********************************************************************/
	/*********************************************************************
	 * Receive Message
	*********************************************************************/
	   
  event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
    call Leds.led1Toggle();    
    return msg;
  }
}

