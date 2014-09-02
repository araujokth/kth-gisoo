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
		
		// stuff for pin
		interface GeneralIO as PinA; //channel A
                interface PinDebug;

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
		
	/*********************************************************************
		* 3) Booting functions and Variable value assignment
	*********************************************************************/

	event void Boot.booted() {
		
		period = 122880;//120*1024;	
		call AMControl.start(); // We need to start the radio
	
	}

	
	void setRelay (uint8_t SIGVal){
	        
		if(SIGVal==1)
                  {        
                    call PinDebug.ADC2set();
                    call Leds.led0On();
                  }
                if(SIGVal==0)
                  {
                  call PinDebug.ADC2clr();
	          call Leds.led0Off();
                  }
                  
          	 busy = FALSE;

           
	}
	
	event void TimerPeriodDT.fired() {
		uint32_t timedelta;
		// if this timer is fired turn the turn relay on
		// set the right value for delta which was received from the NM mote
		call TimerPeriodDT.startPeriodic(period);
		timedelta = period*deltaval/100;
		printf("P: timerperiodfired, timedelta %ld \n", timedelta);printfflush();
		setRelay(1);
		if (deltaval <= 95){
			call TimerDeltaDT.startOneShot(timedelta);
		}
	}
	
	event void TimerDeltaDT.fired() {
		// if this timer is fired we set the relay off
		printf("P: timerdeltafired\n");printfflush();
		setRelay(0);
	}
	
	/*********************************************************************
		* 4) Message reception (MR)
	*********************************************************************/

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
	
		//uint16_t x1;
		//uint16_t x2;
		nx_float x1t; 	
		nx_float x2t; 	
		
		srcid = call AMPacket.source(msg);
		printf("P: srcid %d\n",srcid); printfflush();
	/*********************************************************************
		* 4.1) MR: Define Variables Needed For Controller Implementation
	*********************************************************************/
	
		if ((len == sizeof(xpkt)) && (srcid == 1)) {
			xpkt* svpkt = (xpkt*)payload;
			
			call Leds.led2Toggle();  //toggle led 2 whenever receiving a message
			
			x1t = (svpkt->xval1);
			x2t = (svpkt->xval2);

		}
		if ((len == sizeof(setpkt)) && (srcid == 10)) {
			setpkt* setsvpkt = (setpkt*)payload;
			
			call Leds.led1Toggle();  //toggle led 1 whenever receiving a message
			
			setval = (setsvpkt->setpin);
			deltaval = (setsvpkt->delta);
			printf("P: setval %i, deltaval %i \n",setval,deltaval);printfflush();		
			if (setval == 0){ // if set to stop, set all timers to 0 and disconnect the relay
				setRelay(0);			
				call TimerDeltaDT.stop();
				call TimerPeriodDT.stop();			
			}else{ 
				call TimerPeriodDT.startPeriodic(10);
			}

		}
		
		return msg;
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
		//if ((&pktToActuator == msg)) {
			//call Leds.led1Toggle();
			busy = FALSE;
		//}
	}
	
	
	


}
