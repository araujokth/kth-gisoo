// $Id: RadioCountToLedsC.nc,v 1.6 2008/06/24 05:32:31 regehr Exp $

/*									tab:4
 * "Copyright (c) 2000-2005 The Regents of the University  of California.  
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for any purpose, without fee, and without written agreement is
 * hereby granted, provided that the above copyright notice, the following
 * two paragraphs and the author appear in all copies of this software.
 * 
 * IN NO EVENT SHALL THE UNIVERSITY OF CALIFORNIA BE LIABLE TO ANY PARTY FOR
 * DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES ARISING OUT
 * OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF
 * CALIFORNIA HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * 
 * THE UNIVERSITY OF CALIFORNIA SPECIFICALLY DISCLAIMS ANY WARRANTIES,
 * INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY
 * AND FITNESS FOR A PARTICULAR PURPOSE.  THE SOFTWARE PROVIDED HEREUNDER IS
 * ON AN "AS IS" BASIS, AND THE UNIVERSITY OF CALIFORNIA HAS NO OBLIGATION TO
 * PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS."
 *
 * Copyright (c) 2002-2003 Intel Corporation
 * All rights reserved.
 *
 * This file is distributed under the terms in the attached INTEL-LICENSE     
 * file. If you do not find these files, copies can be found by writing to
 * Intel Research Berkeley, 2150 Shattuck Avenue, Suite 1300, Berkeley, CA, 
 * 94704.  Attention:  Intel License Inquiry.
 */
 
#include "Timer.h"
#include "TestRPL.h"
#include "lib6lowpan/ip.h"

#include "blip_printf.h"


module TestRPLC @safe() {
  uses {
    interface Leds;
    interface Boot;
    interface Timer<TMilli> as MilliTimer;
    interface Timer<TMilli> as Timer;

interface Resource;	//ADCS
interface Msp430Adc12MultiChannel as MultiChannel;


    interface RPLRoutingEngine as RPLRoute;
    interface RootControl;
    interface StdControl as RoutingControl;
    interface SplitControl;
    //interface IP as RPL;
    interface UDP as RPLUDP;
    //interface RPLForwardingEngine;
    interface RPLDAORoutingEngine as RPLDAO;
    interface Random;
    
    //DACS
    interface GeneralIO as ADC0;
    interface GeneralIO as ADC1;


    //interface Lcd;
    //interface Draw;
  }
  provides interface AdcConfigure<const msp430adc12_channel_config_t*>;

}
implementation {

#ifndef RPL_ROOT_ADDR
#define RPL_ROOT_ADDR 1
#endif

#define UDP_PORT 5678
#define SCAL 87.3932
#define ACAL 273

#define REFERENCE1 5
#define REFERENCE2 8

  //uint8_t payload[10];
  //struct in6_addr dest;
  struct in6_addr MULTICAST_ADDR;

  bool locked;
  uint16_t counter = 0;
  uint16_t buffer[3];
  nx_float K[3];
  nx_float ref;
  // actuator
  float xca;
  float x1;
  float x2;
     float xcapre;
    float x1pre;
    float x2pre;
    
  nx_float outf;
  nx_float upre;
  uint16_t u;
  // sensor 
  nx_int16_t xcs;
  nx_int16_t xcarx;
  
  uint32_t period;
  
  event void Boot.booted() {
    memset(MULTICAST_ADDR.s6_addr, 0, 16);
    MULTICAST_ADDR.s6_addr[0] = 0xFF;
    MULTICAST_ADDR.s6_addr[1] = 0x2;
    MULTICAST_ADDR.s6_addr[15] = 0x1A;

    //call Lcd.initialize();

    if(TOS_NODE_ID == RPL_ROOT_ADDR){
      call RootControl.setRoot();
// DACS	
atomic {
	ADC12CTL0 = REF2_5V +REFON;
	DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
	}
	DAC12_0DAT = 0;
    }
    call RoutingControl.start();
    call SplitControl.start();
    call RPLUDP.bind(UDP_PORT);
    
   // K[0] =  -1.1437; 
   // K[1] = -3.3787 ;
   // K[2] = -0.3162;
   K[0] =  -0.9818 ;//-1.1437; 
   K[1] = -2.4590;//-3.3787 ;
   K[2] = -0.3162;//-0.3162;
   //-0.7896   -1.4487   -0.1327
   //-0.5741   -0.9283   -0.0780
   // K[0] = -0.5741 ; // -0.6919   -1.1460   -0.1058    
   // K[1] = -0.9283;  // -1.0808   -2.1970   -0.2364
   // K[2] = -0.0780; // -1.0041   -1.9688   -0.2061
   
    xca = 0;
    xcapre = 0;
    upre = 0;
    ref = REFERENCE1;
    xcs = 0;
    period = 100000;
    call Timer.startPeriodic(period);
    
    if(TOS_NODE_ID == 10){
	call MilliTimer.startOneShot(PACKET_INTERVAL+100); // first time to call the timer set it a bit earlier to compensate for the travelling time
	printf("PRINTPKT: first time. Set timer\n");
    }
    
  }

// ADCS

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Prototype functions

	task void getData();
	void printfFloat(float toBePrinted); // For printing out float numbers

// ADCS

  uint32_t countrx = 0;
  uint32_t counttx = 0;
  uint32_t timek = 1;
 nx_uint16_t tankLevel[2];

 
  event void RPLUDP.recvfrom(struct sockaddr_in6 *from, void *payload, uint16_t len, struct ip6_metadata *meta){

    nx_int16_t temp[5];
    uint32_t tx_count = 0;
    memcpy(temp, (uint8_t*)payload, len);
    call Leds.led2Toggle();
    ++countrx;
 
    
    tx_count = (uint32_t) temp[1];
    // Implement here the estimator and controller
    x1 = temp[2]/SCAL;
    x2 = temp[3]/SCAL;
    //xca = temp[4]/SCAL;

    xca += ((nx_float) (x2 - ref));
    printf("PKTPRINTa");
    printfFloat(x1);
   // printfFloat(x2);
   // printfFloat(xca);
    //outf = (273.0 *( (x1) * K[0] + (x2) * K[1] + x_int * K[2] ));
    
    // estimation
    //x1pre = 0.92*x1 + 0.15*upre;
    //x2pre = 0.74*x1 + 0.94*x2 + 0.006*upre;
    //xcapre = 0.04*x1 + 0.97*x2 + xca + 0.002*upre - ref;
   
   //x1 = x1pre;
   //x2 = x2pre;
   //xca = xcapre;
   
    outf =   ACAL*(x1 * K[0] + x2 * K[1] + xca * K[2]) ;
    upre = outf/ACAL;
    
    if(outf < 0) outf = 0;
    else if(outf > 4095) outf = 4095;
    u = (uint16_t) outf;
    printf("PKTPRINTa: control u %d\n",u);
    atomic {
	DAC12_0DAT = u;
    }
    
    // printf("PRINTPKT: RX of ID %d, vals [%d,%d,%d,%d], delay: %ld, timek %ld, countrx %ld, counttx %d \n", TOS_NODE_ID, temp[2], temp[3],  temp[4],u, timek-tx_count, timek, countrx, temp[1]);
    printf("PKTPRINT: RX at ID %d, pktid %ld, pktlost %ld \n", TOS_NODE_ID, tx_count, tx_count - countrx);
    printfflush();
    
  }
  
  event void SplitControl.startDone(error_t err){
    while( call RPLDAO.startDAO() != SUCCESS );
    
    //if(TOS_NODE_ID != RPL_ROOT_ADDR){
     // call Timer.startOneShot((call Random.rand16()%2)*1024U);
    //}
    // ADCS
    if(TOS_NODE_ID == 1){
      call Resource.request();
    }
    // ADCS
  }

/* Initial timer */
  event void Timer.fired(){
	if (ref == REFERENCE1){
		ref = REFERENCE2;
	}
	else{
		ref = REFERENCE1;
	}
   
  }

/* SEND TASK */
  task void sendTask(){
    struct sockaddr_in6 dest;
	nx_int16_t temp[5];
   
	// temp[0] TOS_NODE_ID
	// temp[1] conttx
	// temp[2] x1
	// temp[3] x2
	// temp[4] xc

    uint8_t i;

   
    memcpy(dest.sin6_addr.s6_addr, call RPLRoute.getDodagId(), sizeof(struct in6_addr));

    if(dest.sin6_addr.s6_addr[15] != 0) // destination is set as root!
      ++counttx;

    call Leds.led0Toggle();

    dest.sin6_port = htons(UDP_PORT);

 for(i=0;i<5;i++){
      temp[i] = 0;
    }
    
    temp[0] = TOS_NODE_ID;
    temp[1] = counttx;
    temp[2] = tankLevel[0];
    temp[3] = tankLevel[1];
    xcs = xcs + (temp[3] - ref*SCAL);
    temp[4] =  xcs;
    
    //printf("PRINTPKT: Generate Packet at %d, value [%d,%d,%d] and count %d \n", TOS_NODE_ID, temp[2], temp[3], temp[4], temp[1] );
    printf("PKTPRINT: TX at ID %d, pktid %ld\n", TOS_NODE_ID, counttx);
    call RPLUDP.sendto(&dest, temp, 20);
  }

/* Periodic timer */
  event void MilliTimer.fired(){ 
	if(TOS_NODE_ID == 1){
		call Leds.led1Toggle();
		call MilliTimer.startOneShot(PACKET_INTERVAL);
		post getData(); // ADC
    		
	}
	if(TOS_NODE_ID == 10){
		//printf("PRINTPKT: increment timek\n");
		timek = timek + 1; // increment the time counter
		call MilliTimer.startOneShot(PACKET_INTERVAL);
	}
  }

  event void SplitControl.stopDone(error_t err){}

/*********************************************************************
	 * Multi Channel functions
	 *********************************************************************/
	task void getData()
	{
		// Read ADC values
		call MultiChannel.getData();
	}

	event void Resource.granted()
	{
		atomic {
			adc12memctl_t memctl[] = { {INPUT_CHANNEL_A0, REFERENCE_VREFplus_AVss}, {INPUT_CHANNEL_A1, REFERENCE_VREFplus_AVss}};

			if (call MultiChannel.configure(&config, memctl, 2, buffer, 3, 0) != SUCCESS) {
				call Leds.led0On();
			}
		}
		if(TOS_NODE_ID != 10){
	call MilliTimer.startOneShot(PACKET_INTERVAL);
	}
 		
	
	}
	async event void MultiChannel.dataReady(uint16_t *buf, uint16_t numSamples)
	{
		// Copy sensor readings
		tankLevel[0] = buf[1];
		tankLevel[1] = buf[2];
	  	

		//printf("PRINTPKT: Generate Packet at %d",tankLevel[0]);

		// send the packet
		post sendTask(); 
	}
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	/*********************************************************************
	 * END Multi Channel functions
	 *********************************************************************/

/*********************************************************************
	 * ADDITIONAL TOOLS (For printing out float values)
	 **********************************************************************/
	void printfFloat(float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		float f = toBePrinted;

		if (f<0) {
			c = '-'; f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((float) fi);
		f0 = f*10; f0 %= 10;
		f1 = f*100; f1 %= 10;
		f2 = f*1000; f2 %= 10;
		printf("PRINTPKT: %c%ld.%d%d%d \n", c, fi, (uint8_t) f0, (uint8_t) f1,
				(uint8_t) f2);
	}

}
