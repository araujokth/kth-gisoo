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

  // actuator
  float xca;
  float x1;
  float x2;
  

nx_float xo[3][1];
nx_float y[3][1];

 
  
  nx_float K[3];
  nx_float ref;
  
nx_float M[3][3];
nx_float Brr[3][1];
nx_float Wn[3][3];
nx_float Wv[2][3];
nx_float Ku[1][3];
nx_float I[3][3];
nx_float g2Q[3][3];
nx_float g2M[3][3];
nx_float DD[3][3];
nx_float CNC[3][3];
nx_float AT[3][3];

uint8_t *ylog[5] = {NULL,NULL,NULL,NULL,NULL};
uint8_t *ulog[5] = {NULL,NULL,NULL,NULL,NULL};
uint8_t *xolog = {NULL};
uint8_t *Slog = {NULL};

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
  // vars
  
    K[0] =  -1.1437; 
    K[1] = -3.3787 ;
    K[2] = -0.3162;
    xca = 0;
    ref = 5;
    xcs = 0;
     
 // actuator fixed defs
   
    
M[0][0] = 19.2772;
M[0][1] = 93.9002;
M[0][2] = 9.0737;
M[1][0] = 93.9002;
M[1][1] = 742.2570;
M[1][2] = 86.3987;
M[2][0] = 9.0737;
M[2][1] = 86.3987;
M[2][2] = 14.5464;

Brr[0][0] = 0.0000;
Brr[1][0] = 0.0000;
Brr[2][0] = -10.0000;

Wn[0][0] = 0.0019;
Wn[0][1] = 0.0086;
Wn[0][2] = 0.0008;
Wn[1][0] = 0.0107;
Wn[1][1] = 0.0825;
Wn[1][2] = 0.0092;
Wn[2][0] = 0.0000;
Wn[2][1] = 0.0001;
Wn[2][2] = 0.0000;

Wv[0][0] = 0.0019;
Wv[0][1] = 0.0086;
Wv[0][2] = 0.0008;
Wv[1][0] = 0.0001;
Wv[1][1] = 0.0008;
Wv[1][2] = 0.0001;

Ku[0][0] = 2.5027;
Ku[0][1] = 12.1939;
Ku[0][2] = 1.1493;

I[0][0] = 1.0000;
I[0][1] = 0.0000;
I[0][2] = 0.0000;
I[1][0] = 0.0000;
I[1][1] = 1.0000;
I[1][2] = 0.0000;
I[2][0] = 0.0000;
I[2][1] = 0.0000;
I[2][2] = 1.0000;

g2Q[0][0] = 0.0001;
g2Q[0][1] = 0.0000;
g2Q[0][2] = 0.0000;
g2Q[1][0] = 0.0000;
g2Q[1][1] = 0.0001;
g2Q[1][2] = 0.0000;
g2Q[2][0] = 0.0000;
g2Q[2][1] = 0.0000;
g2Q[2][2] = 0.0001;

g2M[0][0] = 0.0024;
g2M[0][1] = 0.0115;
g2M[0][2] = 0.0011;
g2M[1][0] = 0.0115;
g2M[1][1] = 0.0906;
g2M[1][2] = 0.0105;
g2M[2][0] = 0.0011;
g2M[2][1] = 0.0105;
g2M[2][2] = 0.0018;

DD[0][0] = 1.0000;
DD[0][1] = 0.0000;
DD[0][2] = 0.0000;
DD[1][0] = 0.0000;
DD[1][1] = 1.0000;
DD[1][2] = 0.0000;
DD[2][0] = 0.0000;
DD[2][1] = 0.0000;
DD[2][2] = 0.0001;

CNC[0][0] = 1.0000;
CNC[0][1] = 0.0000;
CNC[0][2] = 0.0000;
CNC[1][0] = 0.0000;
CNC[1][1] = 10000.0000;
CNC[1][2] = 0.0000;
CNC[2][0] = 0.0000;
CNC[2][1] = 0.0000;
CNC[2][2] = 10000.0000;

AT[0][0] = 0.9608;
AT[0][1] = 0.0386;
AT[0][2] = 0.0000;
AT[1][0] = 0.0000;
AT[1][1] = 0.9693;
AT[1][2] = 1.0000;
AT[2][0] = 0.0000;
AT[2][1] = 0.0000;
AT[2][2] = 1.0000;

  }

// ADCS

	const msp430adc12_channel_config_t config = {
		INPUT_CHANNEL_A5, REFERENCE_VREFplus_AVss, REFVOLT_LEVEL_2_5,
		SHT_SOURCE_SMCLK, SHT_CLOCK_DIV_1, SAMPLE_HOLD_64_CYCLES,
		SAMPCON_SOURCE_SMCLK, SAMPCON_CLOCK_DIV_1
	};

	// Prototype functions

	task void getData();
	
// matrix operations	
void matrix_multiply(uint8_t m, uint8_t p, uint8_t n, nx_float matrix1[m][p], nx_float matrix2[p][n], nx_float output[m][n]);
void matrix_inv33(nx_float a[3][3], nx_float output[3][3]);
void matrix_add(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n]);
//void matrix_print( uint8_t m, uint8_t n, nx_float matrix[m][n]);

	//void printfFloat(float toBePrinted); // For printing out float numbers

// ADCS

  uint32_t countrx = 0;
  uint32_t counttx = 0;
  uint32_t tx_count = 0;
  uint32_t timek = 1;
  nx_uint16_t tankLevel[2];

/*********************************************************************
		* Receiving function at the SINK
*********************************************************************/

 
  event void RPLUDP.recvfrom(struct sockaddr_in6 *from, void *payload, uint16_t len, struct ip6_metadata *meta){

    nx_int16_t temp[5];
    memcpy(temp, (uint8_t*)payload, len);
    call Leds.led2Toggle();
    ++countrx;
   
   if(TOS_NODE_ID == 10 && timek == 1){
	call MilliTimer.startOneShot(PACKET_INTERVAL-delta); // first time to call the timer set it a bit earlier to compensate for the travelling time
	call Timer.startOneShot(deltabar-delta); // call it to process the data
	printf("PRINTPKT1: first reception. Set timer\n");
	printfflush();
    }	
    
    tx_count = (uint32_t) temp[1];
    
    // Log the data just received so we can use it in the timer function
    x1 = temp[2]/SCAL;
    x2 = temp[3]/SCAL;
    xca = temp[4]/SCAL;
    
     printf("PRINTPKT1: RX of ID %d, vals [%d,%d,%d], delay: %ld, timek %ld, countrx %ld, counttx %d \n", TOS_NODE_ID, temp[2], temp[3],  temp[4], timek-tx_count, timek, countrx, temp[1]);
    printfflush();
    
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
   
    memcpy(dest.sin6_addr.s6_addr, call RPLRoute.getDodagId(), sizeof(struct in6_addr));

    if(dest.sin6_addr.s6_addr[15] != 0) // destination is set as root!
      ++counttx;

    call Leds.led0Toggle();

    dest.sin6_port = htons(UDP_PORT);

    temp[0] = TOS_NODE_ID;
    temp[1] = counttx;
    temp[2] = tankLevel[0];
    temp[3] = tankLevel[1];
    xcs = xcs + (temp[3] - ref*SCAL);
    temp[4] =  xcs;

    printf("PRINTPKT1: Generate Packet at %d, value [%d,%d,%d] and count %d \n", TOS_NODE_ID, temp[2], temp[3], temp[4], temp[1] );
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
	if(TOS_NODE_ID == 10){
		printf("PRINTPKT1: BEACON\n");
		printfflush();
		timek = timek + 1; // increment the time counter
		call MilliTimer.startOneShot(PACKET_INTERVAL); // should fire at the same time 
		call Timer.startOneShot(deltabar);
	}
  }

/*********************************************************************
	 * Timer to analyse the data and implement the controller
********************************************************************/
  event void Timer.fired(){
/*
nx_float S[3][3];
nx_float Sg2M[3][3];
nx_float IS[3][3];
nx_float KuIS[1][3];
nx_float Sg2Q[3][3];
nx_float AS[3][3];
nx_float ASxo[3][1];
nx_float Si[3][3];
nx_float Ew[3][1];
nx_float ASNg2Q[3][3];
nx_float ASNCNC[3][3];
nx_float AE[3][3];
nx_float AExo[3][1];
nx_float xob[3][1];

nx_float xo_aux[3][1];
nx_float y_aux[3][1];
nx_float u_aux[1][1];
   
nx_float v[3][1];
nx_float n[3][1];*/

	nx_float outf;
	uint16_t u;
	
	  
    printf("PRINTPKT1: Timer fired. Process data\n");
    printfflush();

	outf =   ACAL*(x1 * K[0] + x2 * K[1] + xca * K[2]) ;
    
    //outf = ACAL*
    
    if(outf < 0) outf = 0;
    else if(outf > 4095) outf = 4095;
    u = (uint16_t) outf;
    
    atomic {
	DAC12_0DAT = u;
    }
   
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
	
	

/*********************************************************************
	 * Matrix functions
********************************************************************/
	
	void matrix_multiply(uint8_t m, uint8_t p, uint8_t n, nx_float matrix1[m][p], nx_float matrix2[p][n], nx_float output[m][n])
{
    uint8_t i, j, k;
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
            output[i][j] = 0.0;
        	
    for (i = 0; i < m; i++)
        for (j = 0; j < p; j++)
            for (k = 0; k < n; k++){
                output[i][k] += matrix1[i][j] * matrix2[j][k];
                }
}

void matrix_add(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n])
{
    uint8_t i,j;
    for(i = 0; i < m ; i++)
    	for(j = 0; j < n; j++)
    		output[i][j] =  (matrix1[i][j] + matrix2[i][j]);
}  


void matrix_inv33(nx_float a[3][3], nx_float output[3][3]){
nx_float determinant;
uint8_t i, j;
determinant = 0;

for(i=0;i<3;i++)
      determinant = determinant + (a[0][i]*(a[1][(i+1)%3]*a[2][(i+2)%3] - a[1][(i+2)%3]*a[2][(i+1)%3]));

   for(i=0;i<3;i++){
      for(j=0;j<3;j++)
           output[i][j] = ((a[(i+1)%3][(j+1)%3] * a[(i+2)%3][(j+2)%3]) - (a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]))/ determinant;
   }
}

/*********************************************************************
	 * ADDITIONAL TOOLS (For printing out float values)
**********************************************************************/
	/*void printfFloat(float toBePrinted) {
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
	}*/
	
	

}
