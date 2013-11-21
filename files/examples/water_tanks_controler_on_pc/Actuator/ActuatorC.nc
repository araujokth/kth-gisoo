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
 * @version  $Revision: 1.0 Date: 2011/10/29 $ 
 */

#include "AM.h"
#include "app_parameters.h"

module ActuatorC {
	uses {
		interface Boot;
		interface Leds;

		interface Receive;
		interface SplitControl as AMControl;

		interface GeneralIO as ADC0;
		interface GeneralIO as ADC1;
	}
}
implementation {
	event void Boot.booted() {

		atomic {
			ADC12CTL0 = REF2_5V +REFON;
			DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
		}



		
		//To not disturb the Sensor Values
		call ADC0.makeInput();
		call ADC1.makeInput();

		DAC12_0DAT = 0;

		call AMControl.start();

	}
	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {
		}
		else {
			call AMControl.start();
		}
	}

	event void AMControl.stopDone(error_t err) {
	}

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		if (len == sizeof(EncMsg2SensorsAct)) {
			EncMsg2SensorsAct* apkt = (EncMsg2SensorsAct*)payload;
			call Leds.led2Toggle();
			atomic {
				DAC12_0DAT = apkt->u[0];

			}
		} 

		return msg;
	}

	

}
