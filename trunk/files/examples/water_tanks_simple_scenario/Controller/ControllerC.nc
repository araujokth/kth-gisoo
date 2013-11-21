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

module ControllerC {
	uses {

	/*********************************************************************
		* 1) Interfaces definition
	*********************************************************************/

		interface Boot;
		interface Leds;


		interface Receive;
		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;

	}
}
implementation {

	/*********************************************************************
		* 2) Variable definition
	*********************************************************************/
	
	uint16_t u;	// control input
	uint16_t u0;	// control input in the end of 1st phase
	
	uint16_t kp;	// proportional gain
	uint16_t ki;	// integral gain	
	uint16_t kd;	// derivative gain

	uint16_t pc;	// proportional part of u
	uint16_t ic;	// integral part of u	
	uint16_t dc;	// derivative part of u
	
	nx_float x_ref;
	nx_float op_point;	// operating point for 1st phase
	nx_float outf;	// auxiliary variable
	uint16_t e;	// output error

	float x1;	// upper tank	
	float x2; 	// lower tank

	uint16_t beta; //  1v in the pump is approx. 273 units in the DAC
	
	// Other variables
	bool busy;
	bool transmitted;
	message_t pktToActuator;
	message_t pktToRelayNode;
	message_t pktBroadcast;

	nx_float x_int;
	
	nx_float K[3];
	uint16_t i_p;

	uint8_t m_state;

	bool relay;
	bool start;


	/*********************************************************************
		* 3) Booting functions and Variable value assignment
	*********************************************************************/

	event void Boot.booted() {
		
		x1 = 0;
		x2 = 0;
		u = 0;
		x_ref = OP_POINT; // defined as OP_POINT for 1st phase, and it will be changed to REFERNENCE in 2nd phase
		beta = 273;
		
		busy = FALSE;
		transmitted = FALSE;


		K[0] =  -1.1437; 
		K[1] = -3.3787 ;
		K[2] = -0.3162;

		call AMControl.start(); // We need to start the radio
	
	}


	/*********************************************************************
		* 4) Message reception (MR)
	*********************************************************************/

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {

		EncMsg2SensorsAct* apkt = (EncMsg2SensorsAct*)(call Packet.getPayload(&pktToActuator, sizeof (EncMsg2SensorsAct)));

	
		
	/*********************************************************************
		* 4.1) MR: Define Variables Needed For Controller Implementation
	*********************************************************************/
	
		
		if ((len == sizeof(SensorValues))) {

			call Leds.led2Toggle();  //toggle led 2 whenever receiving a message
			atomic {
				SensorValues* svpkt = (SensorValues*)payload;
				
				// Store sensor values
				x1 = (svpkt->tankLevel[0])/(WT_CALIBRATION);
				x2 = (svpkt->tankLevel[1])/(WT_CALIBRATION);
				
				x_int += ((nx_float) SAMPLING_PERIOD/1000 * (x2 - x_ref));
				outf = (273.0 *( (x1) * K[0] + (x2) * K[1] + x_int * K[2] ));

						if(outf < 0) outf = 0;
						else if(outf > 4095) outf = 4095;

						u = (uint16_t) outf;
						u0 = u;
						apkt->u = u;
					

	/*********************************************************************
		* 4.2) MR: Controller Implementation
	*********************************************************************/


				if (!busy) {
					if (call AMSend.send(ACTUATOR_ADDRESS, &pktToActuator, sizeof(EncMsg2SensorsAct)) == SUCCESS) {
						busy = TRUE;
					}
				}
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
		if ((&pktToActuator == msg)) {
			call Leds.led1Toggle();
			busy = FALSE;
		}
	}
	/*********************************************************************
	 	* END *
	 **********************************************************************/

}
