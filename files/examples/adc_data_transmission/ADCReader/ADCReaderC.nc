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
 * @author Behdad Aminian <behdad@kth.se>
 * @author David Andreu <daval@kth.se>
 * 
 * @version  $Revision: 1.1 Date: 2013/11/15 $
 */

#include <Timer.h>

module ADCReaderC {

	uses	interface Boot;
	uses	interface Leds;
	uses	interface Timer<TMilli> as TimerSens;

	//ADC  
  	uses  	interface Resource;
   	uses 	interface Msp430Adc12MultiChannel as MultiChannel;
	provides 	interface AdcConfigure<const msp430adc12_channel_config_t*>;

}

implementation {

	uint16_t buffer[3];
	uint16_t SAMPLING_PERIOD = 200;

	//	ADCValues
	nx_uint16_t adcValues[2];


	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Prototype functions

	task void getData();


void setLeds(uint16_t val) {
    if (val & 0x01)
      call Leds.led0Toggle();//led0On();
    else 
      call Leds.led0Off();
    if (val & 0x02)
      call Leds.led1On();
    else
      call Leds.led1Off();
    if (val & 0x04)
      call Leds.led2On();
    else
      call Leds.led2Off();
  }

	/**----------------------------------------------*/
	event void Boot.booted() {

		call Resource.request();
		
	}


	/*********************************************************************
	 * Multi Channel functions
	 *********************************************************************/
	task void getData()
	{
		// Read ADC values
		call MultiChannel.getData();
	}
	event void TimerSens.fired() {
		post getData();
	}
	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) {
				call Leds.led0On();
				call Leds.led1On();
				call Leds.led2On();
			}
		}
		// Start a periodic timer to read the ADC
		call TimerSens.startPeriodic(SAMPLING_PERIOD);
	}
	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		// Copy sensor readings
		adcValues[0] = buf[1];
		adcValues[1] = buf[2];
 setLeds(adcValues[0]);

	}



	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/
	


}
