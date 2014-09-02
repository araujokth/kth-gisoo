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

//#include "blip_printf.h"


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

#define delta 102
#define deltabar 512

  //uint8_t payload[10];
  //struct in6_addr dest;
  struct in6_addr MULTICAST_ADDR;

  bool locked;
  uint16_t buffer[3];

  nx_float ref;
  

  // sensor 
  nx_int16_t xcs;
  
  event void Boot.booted() {
    memset(MULTICAST_ADDR.s6_addr, 0, 16);
    MULTICAST_ADDR.s6_addr[0] = 0xFF;
    MULTICAST_ADDR.s6_addr[1] = 0x2;
    MULTICAST_ADDR.s6_addr[15] = 0x1A;

    //call Lcd.initialize();

    if(TOS_NODE_ID == RPL_ROOT_ADDR){
      call RootControl.setRoot();
      }
    call RoutingControl.start();
    call SplitControl.start();
    call RPLUDP.bind(UDP_PORT);
    
  // vars
  
    ref = 3;
    xcs = 0;
     
  }

// ADCS

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Prototype functions

	task void getData();

  uint32_t countrx = 0;
  uint32_t counttx = 0;
  nx_uint16_t tankLevel[2];

/*********************************************************************
		* Receiving function at the SINK
*********************************************************************/

 void matrix_print( uint8_t m, uint8_t n, nx_float matrix[m][n]);
 void printfFloat(nx_float toBePrinted); // For printing out float numbers

 
  event void RPLUDP.recvfrom(struct sockaddr_in6 *from, void *payload, uint16_t len, struct ip6_metadata *meta){

    nx_int16_t temp[5];
    memcpy(temp, (uint8_t*)payload, len);
    call Leds.led2Toggle();
    ++countrx;
   
  }
  
  event void SplitControl.startDone(error_t err){
    while( call RPLDAO.startDAO() != SUCCESS );
    
    // ADCS
    if(TOS_NODE_ID == 1){
      call Resource.request();
    }
    // ADCS
  }



/*********************************************************************
		* Sending function at the sensor
*********************************************************************/

  task void sendTask(){
    struct sockaddr_in6 dest;
	nx_int16_t temp[5];
   nx_float y[3][1];
  nx_float xca;
  nx_float x1;
  nx_float x2;
   
    memcpy(dest.sin6_addr.s6_addr, call RPLRoute.getDodagId(), sizeof(struct in6_addr));

    if(dest.sin6_addr.s6_addr[15] != 0) // destination is set as root!
      ++counttx;

    call Leds.led0Toggle();

    dest.sin6_port = htons(UDP_PORT);

    temp[0] = TOS_NODE_ID;
    temp[1] = counttx;
    atomic {
    if (counttx <= 5){
    	temp[2] = 0;
    	temp[3] = 0;
    	temp[4] = 0;
    	}
    else{
	    temp[2] = tankLevel[0];
	    temp[3] = tankLevel[1];   
	    temp[4] =  xcs;
	    xcs = xcs + (temp[3] - ref*SCAL)*2;
	   
    }
    
    // Log the data just received so we can use it in the timer function
    x1 = temp[2]/SCAL;
    x2 = temp[3]/SCAL;
    xca = temp[4]/SCAL;

    // read the values to y
    y[0][0] = x1;
    y[1][0] = x2;
    y[2][0] = xca;
    
    printf("P: sensor y\n");
    matrix_print(3,1,y);
           
    }

   // printf("P: XGenerate Packet at %d, value [%d,%d,%d] and count %d \n", TOS_NODE_ID, temp[2], temp[3], temp[4], temp[1] );
   //printf(" %d %d %d\n", temp[2], temp[3], temp[4]);
   printf("P:X %d %d %d\n", temp[2], temp[3], temp[4]);
    printfflush();
    call RPLUDP.sendto(&dest, temp, 20);
  }

/*********************************************************************
	 * MilliTimer - Periodic Beacon to keep track of time!
********************************************************************/
  event void MilliTimer.fired(){ 
	if(TOS_NODE_ID == 1){
		call Leds.led1Toggle();
		call MilliTimer.startOneShot(PACKET_INTERVAL);
		post getData(); // ADC
    		
	}
  }

  event void Timer.fired(){
  }

event void SplitControl.stopDone(error_t err){}

/*********************************************************************
	 * Multi Channel functions - Get ADC data
********************************************************************/
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

		// send the packet
		post sendTask(); 
	}
	async command const msp430adc12_channel_config_t* AdcConfigure.getConfiguration()
	{
		return &config;
	}
	
	void matrix_print( uint8_t m, uint8_t n, nx_float matrix[m][n])
{
	uint8_t i, j;
	for (i = 0; i < m; i++)
        	for (j = 0; j < n; j++)
        		printfFloat(matrix[i][j]);

}
	
 void printfFloat(nx_float toBePrinted) {
		uint32_t fi, f0, f1, f2;
		char c;
		nx_float f;
		
		f = toBePrinted;

		if (f<0) {
			c = '-'; f = -f;
		} else {
			c = ' ';
		}

		// integer portion.
		fi = (uint32_t) f;

		// decimal portion...get index for up to 3 decimal places.
		f = f - ((nx_float) fi);
		f0 = f*10; f0 %= 10;
		f1 = f*100; f1 %= 10;
		f2 = f*1000; f2 %= 10;
		printf("P: %c%ld.%d%d%d \n", c, fi, (uint8_t) f0, (uint8_t) f1,
				(uint8_t) f2);
				
		
	}	


	

}
