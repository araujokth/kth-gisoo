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
#include "app_parameters.h"
#include "app_wt_calibration.h"

module CTPControllerC {

//EasyCollection
  uses interface Boot;
  uses interface SplitControl as RadioControl;
  uses interface StdControl as RoutingControl;
 
  uses interface Leds;
  uses interface RootControl;
  uses interface Receive;
  
  
  uses interface AMSend;
  uses interface Packet;

  uses interface PacketAcknowledgements;
  
}
implementation {
  message_t packet;
  bool sendBusy = FALSE;
  
//Controller
	message_t pktToActuator;
	message_t pktToActuator2;
	message_t tempPktToActuator[40];
	uint16_t flag;
	uint16_t j;
	
	float x1;	// upper tank	
	float x2; 	// lower tank
	nx_float x_int;
	nx_float x_ref;
	nx_float op_point;	// operating point for 1st phase
	nx_float outf;	// auxiliary variable
	nx_float K[3];
	uint16_t u;	// control input
	uint16_t u0;	// control input in the end of 1st phase
	uint16_t e;	// output error
	uint16_t beta; //  1v in the pump is approx. 273 units in the DAC

	uint16_t pc;	// proportional part of u
	uint16_t ic;	// integral part of u	
	uint16_t dc;	// derivative part of u
	bool busy;
	EncMsg2SensorsAct* apkt;
	EncMsg2SensorsAct* apkt2;
	
	uint16_t nRcvPkt[21];
	uint16_t rcvPID[21];
	uint16_t ctrlP_ID[21];	
	SensorValues* svpkt;
	
	uint8_t m_phase;  
  
   uint16_t nRcv[21];
   uint16_t nLost[21];
   uint8_t i;
//Function prototype   
     task void send();

  event void Boot.booted() {
  flag=0;
  j=1;
  	x1 = 0;
	x2 = 0;
	u = 0;
	x_ref = OP_POINT;
	beta = 273;
	
	K[0] =  -1.2109; 
	K[1] = -3.8028;
	K[2] = -0.2879;
	
	busy = FALSE;
	for (i=0;i<21;i++)
   {  
       nRcv[i]=0;
       nLost[i]=0;
       nRcvPkt[i]=0;
       rcvPID[i]=0;
       ctrlP_ID[i]=0;
   }
	
    call RadioControl.start();
  }
  
  event void RadioControl.startDone(error_t err) {
    if (err != SUCCESS)
      call RadioControl.start();
    else {
      call RoutingControl.start();
      call RootControl.setRoot();
    }
  }
  
  event void RadioControl.stopDone(error_t err) {}
/////Controller
/*********************************************************************
*  Message reception (MR)
*********************************************************************/

event message_t* 
Receive.receive(message_t* msg, void* payload, uint8_t len) {
    call Leds.led1Toggle();  

		//EncMsg2SensorsAct* 
		apkt = (EncMsg2SensorsAct*)(call Packet.getPayload(&pktToActuator2, sizeof (EncMsg2SensorsAct)));

	if (len == sizeof(SensorValues)){
	
	atomic {
				svpkt = (SensorValues*)payload;
				
				// Store sensor values
				x1 = (svpkt->tankLevel[0])/(WT_CALIBRATION);
				x2 = (svpkt->tankLevel[1])/(WT_CALIBRATION);
				
				nLost[svpkt->sensor_ID]=svpkt->pkt_ID-nRcv[svpkt->sensor_ID];
				apkt->sensor_ID=svpkt->sensor_ID;
				apkt->pkt_ID=svpkt->pkt_ID;
				apkt->ctrlP_ID=ctrlP_ID[svpkt->sensor_ID];
				nRcv[svpkt->sensor_ID]++;
				

				
						rcvPID[svpkt->sensor_ID]=svpkt->pkt_ID;
						nRcvPkt[svpkt->sensor_ID]=nRcvPkt[svpkt->sensor_ID]++;
						x_int += ((nx_float) SAMPLING_PERIOD/1000 * (x2 - x_ref));
						
						outf = (273.0 *( (x1) * K[0] + (x2) * K[1] + x_int * K[2] ));
						if(outf < 0) outf = 0;
						else if(outf > 4095) outf = 4095;
						
						u = (uint16_t) outf;
						u0 = u;
				  	apkt->u = u;
						
				
				flag++;
				tempPktToActuator[flag]=pktToActuator2;				
				
				if (!busy) {
					pktToActuator=tempPktToActuator[j];
					post send();
				}
			}
			return msg;
  }
  
  }
  
    task void send() {
    
    
    		apkt2 = (EncMsg2SensorsAct*)(call Packet.getPayload(&pktToActuator, sizeof (EncMsg2SensorsAct)));
    call PacketAcknowledgements.requestAck(&pktToActuator);
    if(call AMSend.send(apkt2->sensor_ID+20, &pktToActuator, sizeof(EncMsg2SensorsAct)) != SUCCESS) {
      post send();
      }
    else
    {
    	busy = TRUE;//%%%%%%%%%%%%%%ACTUATOR_ADDRESS
	call Leds.led2Toggle();  //toggle led 2 whenever receiving a message
	ctrlP_ID[svpkt->sensor_ID]=ctrlP_ID[svpkt->sensor_ID]++;
    }
    }
	
		event void AMSend.sendDone(message_t* msg, error_t error) {
		
		if ((&pktToActuator == msg) ) {
		
			if(call PacketAcknowledgements.wasAcked(msg)) {
			
				call Leds.led0Toggle();
				busy = FALSE;
				j++;
				
				if(flag>0 && j<=flag){
					pktToActuator=tempPktToActuator[j];
		
					if(!busy){
						post send();
					}
				
				}
				if (j>flag){
					flag=0;
					j=1;	
				}
			
			}else{
				busy = FALSE;
				post send();
			}
			
			
		}		
  
	}
}

