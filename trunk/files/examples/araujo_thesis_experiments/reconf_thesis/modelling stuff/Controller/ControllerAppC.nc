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

configuration ControllerAppC {
}
implementation {

	components MainC, ControllerC, LedsC;
	components ActiveMessageC;
	components new AMSenderC(AM_CHANNEL);
	components new AMReceiverC(AM_CHANNEL);
	components new TimerMilliC() as TimerPhase;
	components new TimerMilliC() as Timertxbeacon;
	components new TimerMilliC() as Timercheckhealth;
	components new TimerMilliC() as Timertxdist;
	
	
	ControllerC.Boot -> MainC;
	ControllerC.Leds -> LedsC;
	ControllerC.Receive -> AMReceiverC;

	ControllerC.Packet -> AMSenderC;
	ControllerC.AMPacket -> AMSenderC;
	ControllerC.AMSend -> AMSenderC;
	ControllerC.AMControl -> ActiveMessageC;
	ControllerC.TimerPhase -> TimerPhase;
	ControllerC.Timertxbeacon -> Timertxbeacon;
	ControllerC.Timertxdist -> Timertxdist;
	ControllerC.Timercheckhealth-> Timercheckhealth;
	
	
	components SerialPrintfC;
  	components SerialStartC;
	
	components HplMsp430GeneralIOC;
	components new Msp430GpioC() as ADC0;
	ADC0 -> HplMsp430GeneralIOC.Port60;
	ControllerC.ADC0 -> ADC0;

	components new Msp430GpioC() as ADC1;
	ADC1 -> HplMsp430GeneralIOC.Port61;
	ControllerC.ADC1 -> ADC1;
	
	components RandomC;
	ControllerC.Random -> RandomC;
	  

}

