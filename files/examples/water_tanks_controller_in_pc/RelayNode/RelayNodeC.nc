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



#include "app_parameters.h"
#include "app_wt_calibration.h"

module RelayNodeC {
	uses {
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
	bool busy = FALSE;
	message_t pkt;
	
	// Prototype functions
	/*-------------------------------------------------------------------*/
	event void Boot.booted() {
		call AMControl.start(); // We need to start the radio
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
	event void AMSend.sendDone(message_t* msg, error_t error) {
		if (&pkt == msg) {
			call Leds.led1Toggle();
			busy = FALSE;
		}
	}
	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {
		if (len == sizeof(SensorValues)) {
			// Received packet from Sensor node
			SensorValues* svpkt = (SensorValues*)payload;
			SensorValues* rsvpkt = (SensorValues*)(call Packet.getPayload(&pkt, sizeof (SensorValues)));
			call Leds.led2Toggle();

				// Send the packet to the Controller node
				if (!busy) {
					atomic {
						memcpy(rsvpkt, svpkt, sizeof(SensorValues) );
					}
					if (call AMSend.send(COORDINATOR_ADDRESS, &pkt, sizeof(SensorValues)) == SUCCESS) {
						busy = TRUE;
					}

				}

		} 

		return msg;
	}


}
