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

module SensorC {
	uses {
		interface Boot;
		interface Leds;
		interface Timer<TMilli> as TimerSend;
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

	SensorValues sensorValues;

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Prototype functions

	task void getData();
	task void sendPacket();

	/**----------------------------------------------*/
	event void Boot.booted() {

		call TimerSensorBoot.startOneShot(1000);
		
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
				SensorValues* svpkt = (SensorValues*)(call Packet.getPayload(&pkt, sizeof (SensorValues)));
				atomic {
					memcpy(svpkt, &sensorValues, sizeof(SensorValues) );
				}
				if (call AMSend.send(RELAYNODE_ADDRESS, &pkt, sizeof(SensorValues)) == SUCCESS) {
					busy = TRUE;
				}

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
		// Copy sensor readings
		sensorValues.tankLevel[0] = buf[1];
		sensorValues.tankLevel[1] = buf[2];

		post sendPacket();
	}
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/
	

}
