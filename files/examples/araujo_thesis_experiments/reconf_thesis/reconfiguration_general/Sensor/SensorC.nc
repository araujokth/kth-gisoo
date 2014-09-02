/*
 * Copyright (c) 2010, KTH Royal Institute of Technology
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
/**
 * @author David Andreu <daval@kth.se>
 * Modified by Behdad Aminian <behdad@kth.se>
 * 
 * @version  $Revision: 1.2 Date: 2013/11/21 $
 */

#include <Timer.h>

#include "app_parameters.h"
#include "app_wt_calibration.h"
#include <printf.h>

module SensorC {
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as TimerSend;
		interface Timer<TMilli> as TimerPhase;
		interface Timer<TMilli> as TimerSensorBoot;

		interface Resource;
		interface Msp430Adc12MultiChannel as MultiChannel;

		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;

	}
	provides interface AdcConfigure<const msp430adc12_channel_config_t*>;

}
implementation {
	bool busy = FALSE;
	message_t pkt;

	uint16_t buffer[3];

	uint16_t rate = SAMPLING_PERIOD;
	uint16_t phase = 1;
	uint32_t phaseperiod = SAMPLING_PERIOD;
	
	nx_float xint;
	uint16_t xref;
	uint16_t buffread;
	uint32_t phaseC;
	
	xpkt xValues;

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Prototype functions

	task void getData();
	task void sendPacket();
	//void printfFloat(nx_float toBePrinted); // For printing out float numbers
	/**----------------------------------------------*/
	event void Boot.booted() {

		call TimerSensorBoot.startOneShot(1024);
		call TimerPhase.startPeriodic(phaseperiod);
	
		xint = 700;
		atomic{
			xref = REFERENCE2;
		}
		phaseC = 0;
		phase = 1;
		
	}
	event void TimerSensorBoot.fired() {
		call AMControl.start();
	}

	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {
			call Resource.request();
		}
		else {
			call AMControl.start();
		}
	}
	event void AMControl.stopDone(error_t err) {
	}

	task void sendPacket() {

			if (!busy) {
				xpkt* svpkt = (xpkt*)(call Packet.getPayload(&pkt, sizeof (xpkt)));
				atomic {
					memcpy(svpkt, &xValues, sizeof(xpkt) );
				}
				if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(xpkt)) == SUCCESS) {
					busy = TRUE;
				}

			}

	}
	
	event void TimerPhase.fired() {
		
		
		phaseC = phaseC + 1;
		if (phaseC == 200){		
			if (phase == 1){
				printf("C: in phase 1 changing to phase 2\n");
				atomic{
				xref = REFERENCE1;}
			
				phase = 2;
			}
			else{
				printf("C: in phase 2 changing to phase 1\n");
				atomic{
				xref = REFERENCE2;}
				
				phase = 1;
			}
			phaseC = 0;
		}
		
	}
	
	event void AMSend.sendDone(message_t* msg, error_t error) {
		if (&pkt == msg) {
			call Leds.led1Toggle();
			busy = FALSE;
		}
	}

	/*********************************************************************
	 * Multi Channel functions
	 *********************************************************************/
	task void getData()
	{
		// Read ADC values
		call MultiChannel.getData();
	}
	event void TimerSend.fired() {
		post getData();
	}
	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) {
				call Leds.led0On();
			}
		}
		// Start a periodic timer to read the ADC
		call TimerSend.startPeriodic(rate);
	}
	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		xValues.xval1 = buf[1];
		xValues.xval2 = xref;
		post sendPacket();
	}
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/
/*void printfFloat(nx_float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		nx_float f;
		
		f = toBePrinted;

		if (f<0) {
			c = '-'; f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((nx_float) fi);
		f0 = f*10; f0 %= 10;
		f1 = f*100; f1 %= 10;
		f2 = f*1000; f2 %= 10;
		printf("P: %c%ld.%d%d%d \n", c, fi, (uint8_t) f0, (uint8_t) f1,
				(uint8_t) f2);
				
		
	}*/

}
