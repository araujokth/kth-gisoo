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
 * @author Aitor Hernandez <aitorhh@kth.se>
 * @author Joao Faria <jfff@kth.se>
 * @author David Andreu <daval@kth.se>
 * Modified by Behdad Aminian <behdad@kth.se>
 *
 * @version  $Revision: 1.0 Date: 2010/11/03 $
 * @modified 2011/10/29
 */

#ifndef APP_PARAMETERS_H
#define APP_PARAMETERS_H

/*********************************************************************
		* Define reference values for LAB 3
*********************************************************************/

#define OP_POINT 5.0		// Operating point(1st phase of the experiment)
#define REFERENCE 8.0		// Reference used in the 2nd phase of the experiment
//#define PRINTFENABLED 1

/*********************************************************************
		* Define values for LAB 3
*********************************************************************/


enum {
	AM_CHANNEL 		= 6, 		// Frequency channel used
	SAMPLING_PERIOD 	= 1024UL,	// 200 ms periodic sampling
	REFERENCE1 		= 17,
	REFERENCE2 		= 20
};

/*********************************************************************
		* Other variables
*********************************************************************/


enum {

	SENSOR_ADDRESS 		=	201,
	RELAYNODE_ADDRESS 	= 	202,
	COORDINATOR_ADDRESS	=	203,
	ACTUATOR_ADDRESS 	= 	204	
};


typedef nx_struct beaconpkt{						
	nx_uint8_t beacon_trigger;
	nx_uint8_t beacon_nn;
	nx_int32_t beacon_mu;
	nx_int32_t beacon_eta0;
	nx_int32_t beacon_eta1;	
}beaconpkt;

typedef nx_struct distpkt{					
	nx_int32_t distvals[3];
}distpkt;

// Type of message send by the sensor node to the relay node
typedef struct xpkt{
	nx_uint16_t xval1;
	nx_uint16_t xval2;
}xpkt;

typedef struct setpkt{
	nx_uint16_t setpin;
	nx_uint16_t delta;
}setpkt;

#endif
