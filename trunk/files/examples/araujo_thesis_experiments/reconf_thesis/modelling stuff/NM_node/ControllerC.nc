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
 * @author Jose Araujo <araujo@kth.se>
 * Modified by Behdad Aminian <behdad@kth.se>
 * @version  $Revision: 1.0 Date: 2011/12/3 $ 
 */



#include "app_parameters.h"
#include "app_wt_calibration.h"
#include <printf.h>
#include <UserButton.h>

module ControllerC {
	uses {



		interface Boot;
		interface Leds;

		interface Timer<TMilli> as TimerPeriodDT;
		interface Timer<TMilli> as TimerDeltaDT;
		
		interface Receive;
		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;
		
                
                // stuff for NM node
                interface Get<button_state_t>;
		interface Notify<button_state_t>;

	}
}
implementation {


	
	uint8_t srcid;
	
	// Other variables
	bool busy;
	bool transmitted;
	
	
	bool relay;
	bool start;

	uint32_t period;
	
	
	uint16_t setval;
	uint16_t deltaval;
	
	setpkt setValues;
	
	message_t pkt;
	
	task void sendPacket();
		
	/*********************************************************************
		* 3) Booting functions and Variable value assignment
	*********************************************************************/

	event void Boot.booted() {
		
		call AMControl.start(); // We need to start the radio
		call Notify.enable();
		period = 1024;	
		call TimerPeriodDT.startPeriodic(period);
		setValues.setpin = 0;
		setValues.delta = 0;
		
		// select the delta for the duty cycle
		deltaval = 50;
		
		
		busy = FALSE;
	}

	
	
	event void TimerPeriodDT.fired() { // this timer is just to check that things are OK
		printf("P: setpin %i, deltaval %i \n",setValues.setpin,setValues.delta);printfflush();		
	}
	
	event void TimerDeltaDT.fired() {
		
	}
	
	event void Notify.notify( button_state_t state ) {
		if ( state == BUTTON_PRESSED ) {
			call Leds.led1On();
			if (setValues.setpin == 0){
				setValues.setpin = 1;
				setValues.delta = deltaval;	
			}else{
				setValues.setpin = 0;
				setValues.delta = 0;	
			}
			// send data packet
			post sendPacket();
		
		} else {
			call Leds.led1Off();
		}
	}
	
	/*********************************************************************
		* 4) Message reception (MR)
	*********************************************************************/

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		
		return msg;
	}

	
	task void sendPacket() {

			if (!busy) {
				setpkt* svpkt = (setpkt*)(call Packet.getPayload(&pkt, sizeof (setpkt)));
				atomic {
					memcpy(svpkt, &setValues, sizeof(setpkt) );
				}
				if (call AMSend.send(AM_BROADCAST_ADDR, &pkt, sizeof(setpkt)) == SUCCESS) {
					busy = TRUE;
				}

			}

	}
		

	/*********************************************************************
	 	* 9) Message functions
	 **********************************************************************/

	event void AMControl.startDone(error_t err) {
		if (err == SUCCESS) {

		}
		else {
			call AMControl.start();
		}
	}

	event void AMControl.stopDone(error_t err) {
	}

	event void AMSend.sendDone(message_t* msg, error_t error) {
		if ((&pkt == msg)) {
			call Leds.led2Toggle();
			busy = FALSE;
		}
	}
	
	
	


}
