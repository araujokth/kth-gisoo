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
/**
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @author Joao Faria <jfff@kth.se>
 * @author David Andreu <daval@kth.se>
 *
 * @modified 2013/11/22
 */

#ifndef APP_PARAMETERS_H
#define APP_PARAMETERS_H
/*********************************************************************
		* Define reference values for LAB 3
*********************************************************************/
#define OP_POINT 5.0		// Operating point(1st phase of the experiment)
#define REFERENCE 10.0		// Reference used in the 2nd phase of the experiment
/*********************************************************************
		* Define values for LAB 3
*********************************************************************/
enum {
	AM_CHANNEL 		= 15, 		// Frequency channel used
	SAMPLING_PERIOD 	= 1000 		// 200 ms periodic sampling
};
/*********************************************************************
		* Other variables
*********************************************************************/
// Type of message send by the controller to the actuator node
typedef nx_struct EncMsg2SensorsAct {
	nx_int16_t 	u;		// Actuation value send to the actuator
	nx_uint8_t 	wtId;		// (Not used)
	nx_uint8_t	sensor_ID;	// NodeID of creator of this packet
	nx_uint16_t	pkt_ID;		// Number of sent packet
	nx_uint16_t	ctrlP_ID;
}EncMsg2SensorsAct;
// Type of message send by the sensor node to the relay node
typedef struct SensorValues {
	nx_uint16_t tankLevel[2]; 		// Values for the tk1 and tk2
	nx_uint8_t 	       sensor_ID;	// NodeID of creator of this packet
	nx_uint16_t	       pkt_ID;		// Number of sent packet
} SensorValues;
#endif
