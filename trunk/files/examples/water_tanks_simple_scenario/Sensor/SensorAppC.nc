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

#include "Msp430Adc12.h"
#include <Timer.h>

configuration SensorAppC {
}
implementation {

	components MainC, SensorC, LedsC, new TimerMilliC() as TimerSend;
	components ActiveMessageC;
	components new TimerMilliC() as TimerSensorBoot;
	components new AMSenderC(AM_CHANNEL);


	SensorC.Boot -> MainC;
	SensorC.Leds -> LedsC;
	SensorC.TimerSensorBoot -> TimerSensorBoot;

	SensorC.TimerSend -> TimerSend;

	SensorC.Packet -> AMSenderC;
	SensorC.AMPacket -> AMSenderC;
	SensorC.AMSend -> AMSenderC;
	SensorC.AMControl -> ActiveMessageC;


	/****************************************
	 * MultiChannel
	 *****************************************/

	components new Msp430Adc12ClientAutoRVGC() as AutoAdc;
	SensorC.Resource -> AutoAdc;
	AutoAdc.AdcConfigure -> SensorC;
	SensorC.MultiChannel -> AutoAdc.Msp430Adc12MultiChannel;

}

