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

}
implementation {

#ifndef RPL_ROOT_ADDR
#define RPL_ROOT_ADDR 1
#endif

#define UDP_PORT 5678
#define SCAL 87.3932
#define ACAL 273


#define delta 100
#define deltabar 500
#define MAX_DELAY 6
#define MAX_DELAY_REAL 5

  //uint8_t payload[10];
  //struct in6_addr dest;
  struct in6_addr MULTICAST_ADDR;

  bool locked;
  uint16_t buffer[3];

  // actuator
  nx_float xca;
  nx_float x1;
  nx_float x2;
  
  nx_float u0;
    
  
  nx_float K[3];
  nx_float ref;
  
nx_float M[3][3];
nx_float Brr[3][1];
nx_float Wn[3][3];
nx_float Wv[3][3];
nx_float Ku[1][3];
nx_float I[3][3];
nx_float g2Q[3][3];
nx_float g2M[3][3];
nx_float DD[3][3];
nx_float CNC[3][3];
nx_float AT[3][3];
nx_float A[3][3];
nx_float B[3][1];

nx_float *ylog[MAX_DELAY] = {NULL,NULL,NULL,NULL,NULL,NULL};
nx_float *ulog[MAX_DELAY] = {NULL,NULL,NULL,NULL,NULL,NULL};
nx_float *xolog[MAX_DELAY] = {NULL,NULL,NULL,NULL,NULL,NULL};
nx_float *Slog[MAX_DELAY] = {NULL,NULL,NULL,NULL,NULL,NULL};
uint8_t pktlog[MAX_DELAY];

uint8_t bufferk_time = 0;


// things needed to be used in the next k step
nx_float xo[3][1];
nx_float S[3][3];


uint8_t rxflag = 0;
uint8_t npkts_rx = 0;
 
  // sensor 
  nx_int16_t xcs;
  
  event void Boot.booted() {
    uint8_t i;
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
 
    xca = 0;

   
    
     
         
 // actuator fixed defs
   
    
M[0][0] = 14.0904;
M[0][1] = 67.2416;
M[0][2] = 6.9628;
M[1][0] = 67.2416;
M[1][1] = 885.0478;
M[1][2] = 114.8260;
M[2][0] = 6.9628;
M[2][1] = 114.8260;
M[2][2] = 24.1699;

Brr[0][0] = 0.0000;
Brr[1][0] = 0.0000;
Brr[2][0] = -6.0000;

Wv[0][0] = 0.0004;
Wv[0][1] = 0.0012;
Wv[0][2] = 0.0001;
Wv[1][0] = 0.0039;
Wv[1][1] = 0.0481;
Wv[1][2] = 0.0055;
Wv[2][0] = 0.0000;
Wv[2][1] = 0.0008;
Wv[2][2] = 0.0001;

Ku[0][0] = 2.4474;
Ku[0][1] = 12.6327;
Ku[0][2] = 1.2207;

I[0][0] = 1.0000;
I[0][1] = 0.0000;
I[0][2] = 0.0000;
I[1][0] = 0.0000;
I[1][1] = 1.0000;
I[1][2] = 0.0000;
I[2][0] = 0.0000;
I[2][1] = 0.0000;
I[2][2] = 1.0000;

g2Q[0][0] = 0.0003;
g2Q[0][1] = 0.0000;
g2Q[0][2] = 0.0000;
g2Q[1][0] = 0.0000;
g2Q[1][1] = 0.0003;
g2Q[1][2] = 0.0000;
g2Q[2][0] = 0.0000;
g2Q[2][1] = 0.0000;
g2Q[2][2] = 0.0003;

g2M[0][0] = 0.0010;
g2M[0][1] = 0.0047;
g2M[0][2] = 0.0005;
g2M[1][0] = 0.0047;
g2M[1][1] = 0.0615;
g2M[1][2] = 0.0080;
g2M[2][0] = 0.0005;
g2M[2][1] = 0.0080;
g2M[2][2] = 0.0017;

DD[0][0] = 0.2428;
DD[0][1] = 0.0096;
DD[0][2] = 0.0000;
DD[1][0] = 0.0096;
DD[1][1] = 0.0004;
DD[1][2] = 0.0000;
DD[2][0] = 0.0000;
DD[2][1] = 0.0000;
DD[2][2] = 0.0000;

CNC[0][0] = 1.0000;
CNC[0][1] = 0.0000;
CNC[0][2] = 0.0000;
CNC[1][0] = 0.0000;
CNC[1][1] = 1.0000;
CNC[1][2] = 0.0000;
CNC[2][0] = 0.0000;
CNC[2][1] = 0.0000;
CNC[2][2] = 100.0000;

AT[0][0] = 0.9231;
AT[0][1] = 0.0745;
AT[0][2] = 0.0000;
AT[1][0] = 0.0000;
AT[1][1] = 0.9395;
AT[1][2] = 2.0000;
AT[2][0] = 0.0000;
AT[2][1] = 0.0000;
AT[2][2] = 1.0000;

A[0][0] = 0.9231;
A[0][1] = 0.0000;
A[0][2] = 0.0000;
A[1][0] = 0.0745;
A[1][1] = 0.9395;
A[1][2] = 0.0000;
A[2][0] = 0.0000;
A[2][1] = 2.0000;
A[2][2] = 1.0000;

B[0][0] = 0.2845;
B[1][0] = 0.0113;
B[2][0] = 0.0000;

xo[0][0] = 0.0000;
xo[1][0] = 0.0000;
xo[2][0] = 0.0000;

S[0][0] = 0.6545;
S[0][1] = 0.5430;
S[0][2] = 0.2433;
S[1][0] = 0.5430;
S[1][1] = 0.5187;
S[1][2] = 0.1871;
S[2][0] = 0.2433;
S[2][1] = 0.1871;
S[2][2] = 0.0938;

u0 = 0.6324;

 for(i=0;i<MAX_DELAY;i++){
      pktlog[i] = 0;
      Slog[i] = malloc(sizeof(S));
      memcpy(Slog[i] , S, sizeof(S));
    }
    
  }

	
// matrix operations	

void matrix_multiply(uint8_t m, uint8_t p, uint8_t n, nx_float matrix1[m][p], nx_float matrix2[p][n], nx_float output[m][n]);
void matrix_inv33(nx_float a[3][3], nx_float output[3][3]);
void matrix_add(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n]);
void matrix_sub(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n]);
void matrix_eq(uint8_t m, uint8_t n, nx_float matrix1[m][n],  nx_float output[m][n]);
void matrix_print( uint8_t m, uint8_t n, nx_float matrix[m][n]);
void printfFloat(nx_float toBePrinted); // For printing out float numbers
void printfFloatU(nx_float toBePrinted); 

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
    int8_t delay;
    uint8_t buffery_index;
    nx_float y[3][1];
    nx_float v[3][1];
    
    memcpy(temp, (uint8_t*)payload, len);
    
    ++countrx;
    
     // Check the packet number
    tx_count = (uint32_t) temp[1];
   
   if(TOS_NODE_ID == 10 && timek == 1){
	call MilliTimer.startOneShot(PACKET_INTERVAL-delta); // first time to call the timer set it a bit earlier to compensate for the travelling time
	call Timer.startOneShot(deltabar-delta); // call it to process the data
	printf("P: Xfirst reception. Set timer\n");
	printfflush();
	
	timek = tx_count;
    }	
   
    
    // Log the data just received so we can use it in the timer function
    x1 = temp[2]/SCAL;
    x2 = temp[3]/SCAL;
    xca = temp[4]/SCAL;

    // read the values to y
    y[0][0] = x1;
    y[1][0] = x2;
    y[2][0] = xca;
    
    // add measurement noise
    //printf("P: y pre v\n");		
    //matrix_print(3,1,y);
    
    matrix_multiply(3,3,1,Wv,y,v);
    matrix_add(3,1,y,v,y);
    
    //printf("P: v\n");		
    //matrix_print(3,1,v);
    
 ////  printf("P: y pos v\n");		
 ////   matrix_print(3,1,y);
    
    // Log the data in the ylog buffer
    // calculate the delay
    delay = timek-tx_count;
    //delay = 0;
    if(delay > MAX_DELAY || delay < 0){ // check if the delay is valid!
    	call Leds.led2Toggle();
    	delay = 0;}
    	
    //if(bufferk_time-delay > 0){
    printf("P: bufferk_time %d\n", bufferk_time);
    buffery_index = ((timek-1)-delay) % MAX_DELAY;
    printf("P: buffery_index %d\n", buffery_index);
    //else{
    //	buffery_index = (bufferk_time) % MAX_DELAY;}
    
		
    // logs
    if(ylog[buffery_index] == NULL){
    	ylog[buffery_index] = malloc(sizeof(y));}
    	
    // log y
    
////    printf("P: y\n");
////    matrix_print(3,1,y);
    memcpy(ylog[buffery_index] , y, sizeof(y));
    // set that you have a msg in that position
    pktlog[buffery_index] = 1;
    
    
    memcpy(y, ylog[buffery_index], sizeof(y)); 
    
    // flag to know if a packet is out of order or not
    if(delay != 0){ //delay
    	rxflag = 1;
    }else{ 	    // no delay
    	rxflag = 2; 
    }
    // log number of packets received in this superframe	
    npkts_rx = npkts_rx + 1;
    	
    // just print stuff
     printf("T %ld %ld %d \n",  timek-tx_count, timek, temp[1]);
    printfflush();
    
  }
  
  event void SplitControl.startDone(error_t err){
    while( call RPLDAO.startDAO() != SUCCESS );
   
  }

/*********************************************************************
	 * MilliTimer - Periodic Beacon to keep track of time!
********************************************************************/
  event void MilliTimer.fired(){ 
  	//uint8_t MD_index;
	if(TOS_NODE_ID == 10){
		
		timek = timek + 1; // increment the time counter
		call MilliTimer.startOneShot(PACKET_INTERVAL); // should fire at the same time 
		call Timer.startOneShot(deltabar);
		
		//if(timek != 1){			
		bufferk_time = (bufferk_time + 1) % MAX_DELAY;
		//}

		printf("P: XBEACON bufferk_time %d, timek %ld\n", bufferk_time, timek-1);
		printfflush();
		
	//	MD_index = ((timek - 1) - MAX_DELAY_REAL) % MAX_DELAY;
    		// clean the pktlog for the old stuff
    		
	}
  }

/*********************************************************************
	 * Timer to analyse the data and implement the controller
********************************************************************/
  
 
 task void processData() {
 
nx_float Sg2M[3][3];
nx_float IS[3][3];
nx_float KuIS[1][3];
nx_float Sg2Q[3][3];
nx_float AS[3][3];
nx_float ASxo[3][1];
nx_float Bu[3][1];
nx_float Si[3][3];
nx_float ASNg2Q[3][3];
nx_float ASNCNC[3][3];
nx_float AE[3][3];
nx_float AExo[3][1];
nx_float xob[3][1];
nx_float xoa[3][1];
nx_float Sa[3][3];

nx_float y[3][1];

nx_float xo_aux[3][1];
nx_float y_aux[3][1];
nx_float u_aux[1][1];
   

nx_float uoutf[1][1];
nx_float outf;
nx_float du;
uint16_t u;

uint8_t bufferk_index;
uint8_t MD_index;

uint8_t kk;
uint8_t bufferkk_index;

   
uint8_t val;

    bufferk_index = (bufferk_time) % MAX_DELAY;	
    MD_index = ((timek-1) - MAX_DELAY_REAL) % MAX_DELAY;
    // clean the pktlog for the old stuff
   	
   
    printf("P: XTimer fired. Process data %d \n", bufferk_time);
   
    printf("P: MD_index %d\n", MD_index);
   
 ////   printf("P: xo\n");		
 ////   matrix_print(3,1,xo);
    
 ////   printf("P: S\n");		
 ////   matrix_print(3,3,S);
    // Controller implementation   
    //Sg2M = S*g2M;
    matrix_multiply(3,3,3,S,g2M,Sg2M); 
   // printf("P: Sg2M\n");
   // matrix_print(3,3,Sg2M);
    //Sg2M = I - Sg2M;
    matrix_sub(3,3,I,Sg2M,Sg2M);
    //IS = (Sg2M)^(-1);
   // printf("P: Sg2M\n");		
   // matrix_print(3,3,Sg2M);
    matrix_inv33(Sg2M,IS);
    //KuIS = Ku*IS;
    matrix_multiply(1,3,3,Ku,IS,KuIS);
    //u = -KuIS*xo;
    matrix_multiply(1,3,1,KuIS,xo,uoutf);
	
    //printf("P: IS\n");		
    //matrix_print(3,3,IS);
    //printf("P: KuIS\n");		
    //matrix_print(1,3,KuIS);
   
    //matrix_print(1,3,KuIS); 	
    
    
    outf =  uoutf[0][0];
    outf = -ACAL*outf;
    
   du = outf + ACAL*u0;
    //printf("P: outf\n");
    //printfFloat(outf);
    if(du < 0) du = 0;
    else if(du > 4095) du = 4095;
    
    //outf = du - ACAL*u0; % this is implemented outside
    u = (uint16_t) du;//outf;
   
    atomic {
	DAC12_0DAT = u;
    }

    du = du - ACAL*u0;
    printfFloatU(du);
    outf = (nx_float) u/ACAL;
      // log the u used now
    uoutf[0][0] = outf;
 ////   printf("P: uoutf\n");		
 ////   matrix_print(1,1,uoutf);
    
    	if(ulog[bufferk_index ]  == NULL){
    	ulog[bufferk_index ] = malloc(sizeof(uoutf));}
    
    memcpy(ulog[bufferk_index ] , uoutf, sizeof(uoutf));
         
    // save xo
	if(xolog[bufferk_index ]  == NULL){
    	xolog[bufferk_index ] = malloc(sizeof(xo));}
    
    memcpy(xolog[bufferk_index ] , xo, sizeof(xo));
	
    // save S
    	if(Slog[bufferk_index]  == NULL){
    	Slog[bufferk_index ] = malloc(sizeof(S));}
    
    memcpy(Slog[bufferk_index ] , S, sizeof(S));
    
    
    
    // Estimator now
   
 ////  for(val=0;val<MAX_DELAY;val++){
////   	memcpy(y, ylog[val], sizeof(y)); 
 ////       printf("P: ylog position %d\n",val);
 ////   	matrix_print(3,1,y);
 ////  }
 if (timek-1 <= 4){
 
 	printf("P: done estimator + controller\n");
  	rxflag = 0;
    	npkts_rx = 0;
    // clear pktlog for the old position
    pktlog[MD_index] = 0;}
 else{
    if(rxflag == 0){ // no packet received, run open loop
    
    printf("P: no pkt rx\n");
    	//Sg2Q = S*g2Q;
    	matrix_multiply(3,3,3,S,g2Q,Sg2Q); 
        //Sg2Q = I - Sg2Q;
        matrix_sub(3,3,I,Sg2Q,Sg2Q);
        //IS = Sg2Q^(-1);
        matrix_inv33(Sg2Q,IS);
        //AS = A*IS;
        matrix_multiply(3,3,3,A,IS,AS); 
        //ASxo = AS*xo;
        matrix_multiply(3,3,1,AS,xo,ASxo);
        //Bu = B*u;
        matrix_multiply(3,3,1,B,uoutf,Bu);
        //xo = ASxo + Bu;
        matrix_add(3,1,ASxo,Bu,xoa);
        //xo = xo + Brr;
        matrix_add(3,1,xoa,Brr,xo);
        
        //Si = S^(-1);
        matrix_inv33(S,Si);
        //Si = Si - g2Q;
        matrix_sub(3,3,Si,g2Q,Si);
        //Sg2Q = Si^(-1);
        matrix_inv33(Si,Sg2Q);
        //AS = A*Sg2Q;
        matrix_multiply(3,3,3,A,Sg2Q,AS); 
        //S = AS*A';
        matrix_multiply(3,3,3,AS,AT,Sa); 
        //S = S + DD;
        matrix_add(3,3,Sa,DD,S);
        
 ////      printf("P: xo\n");
////    	matrix_print(3,1,xo);
    	    
        // reset flags and npkts_rx
        rxflag = 0;
        npkts_rx = 0;

    }else{
    	// get the values at k-MAX_DELAY-1 to be used if needed
    	if(npkts_rx == 1 && rxflag == 2){ // no delay
    	
    	printf("P: no delay, 1 pkt\n");
    	    //Ew = E*wlog(:,k);
            memcpy(y, ylog[bufferk_index], sizeof(y)); 
   ////        printf("P: ylog\n");
   	    matrix_print(3,1,y);
            //Si = S^(-1);
            matrix_inv33(S,Si);
            //Si = Si + CNC;
            matrix_add(3,3,Si,CNC,Si);
            //Si = Si - g2Q;
            matrix_sub(3,3,Si,g2Q,Si);
            //Sg2Q = Si^(-1);
            matrix_inv33(Si,Sg2Q);
            //AS = A*Sg2Q;
            matrix_multiply(3,3,3,A,Sg2Q,AS); 
            //ASNg2Q = AS*g2Q;
            matrix_multiply(3,3,3,AS,g2Q,ASNg2Q); 
            //ASNg2Q = A + ASNg2Q;
            matrix_add(3,3,A,ASNg2Q,ASNg2Q);
            //ASNCNC = AS*CNC;
            matrix_multiply(3,3,3,AS,CNC,ASNCNC); 
            //AE = ASNg2Q - ASNCNC;
            matrix_sub(3,3,ASNg2Q,ASNCNC,AE);
            //AExo =  AE*xo;
            matrix_multiply(3,3,1,AE,xo,AExo);
            //Bu = B*u;
            matrix_multiply(3,3,1,B,uoutf,Bu);
            //xob = ASNCNC*y;
            matrix_multiply(3,3,1,ASNCNC,y,xob);
            //xo = AExo + Bu; 
            matrix_add(3,1,AExo,Bu,xoa);
            //xo = xo + xob;
            matrix_add(3,1,xoa,xob,xoa);
            //xo = xo + Brr;
            matrix_add(3,1,xoa,Brr,xo);
            //S = AS*A';
            
////            printf("P: pkt no delay xo\n");		
////    	    matrix_print(3,1,xo);
    
            matrix_multiply(3,3,3,AS,AT,Sa); 
            //S = S + DD;
            matrix_add(3,3,Sa,DD,S);
	
	}
	else{ // delay or out of order
		
		printf("P: delay or out of order, npkts_rx %d\n",npkts_rx);
/*		
		for(val=0;val<MAX_DELAY;val++){
		   	memcpy(xo_aux, xolog[val], sizeof(xo_aux)); 
			printf("P: xo_aux position %d\n",val);
		    	matrix_print(3,1,xo_aux);
		}
		
		for(val=0;val<MAX_DELAY;val++){
		   	memcpy(S, Slog[val], sizeof(S)); 
			printf("P: S_aux position %d\n",val);
		    	matrix_print(3,3,S);
		}
		
		for(val=0;val<MAX_DELAY;val++){
		   	memcpy(u_aux, ulog[val], sizeof(u_aux)); 
			printf("P: u_aux position %d\n",val);
		    	matrix_print(1,1,u_aux);
		}
		
		for(val=0;val<MAX_DELAY;val++){
			printf("P: pktlog position %d\n",val);
		    	printf("P: pktlog = %d\n", pktlog[val]);
		}
*/	
		memcpy(S,Slog[MD_index],sizeof(S));
		memcpy(xo_aux,xolog[MD_index],sizeof(xo_aux));	
   
		printf("P: xo_aux\n");		
		    matrix_print(3,1,xo_aux);
		    
		    printf("P: S\n");		
		    matrix_print(3,3,S);
    
		for(kk=(timek-1)-MAX_DELAY_REAL;kk<=(timek-1);kk++){
		
		
		
			bufferkk_index = kk % MAX_DELAY;		
			memcpy(u_aux,ulog[bufferkk_index],sizeof(u_aux));
	////		printf("P: u_aux, kkindex %d\n", bufferkk_index);		
	////	    	matrix_print(1,1,u_aux);
		   		    	
		       if(Slog[bufferkk_index]  == NULL){
    		       Slog[bufferkk_index ] = malloc(sizeof(S));}
    		       memcpy(Slog[bufferkk_index ] , S, sizeof(S));
    		       
    		       if(xolog[bufferkk_index]  == NULL){
    		       xolog[bufferkk_index ] = malloc(sizeof(xo));}
    		       memcpy(xolog[bufferkk_index ] , xo_aux, sizeof(xo));
    		       
    		       
    		       if (kk <= 4){
		
		       }
		       else{
		
		
			if( pktlog[bufferkk_index] == 1){ // this means a packet is to be read aat this location			
				//Ew = E*wlog(:,k);
			    memcpy(y_aux,ylog[bufferkk_index], sizeof(y_aux)); 
	////		    printf("P: y_aux\n");		
	////	    	    matrix_print(3,1,y_aux);
			    //Si = S^(-1);
			    matrix_inv33(S,Si);
			    //Si = Si + CNC;
			    matrix_add(3,3,Si,CNC,Si);
			    //Si = Si - g2Q;
			    matrix_sub(3,3,Si,g2Q,Si);
			    //Sg2Q = Si^(-1);
			    matrix_inv33(Si,Sg2Q);
			    //AS = A*Sg2Q;
			    matrix_multiply(3,3,3,A,Sg2Q,AS); 
			    //ASNg2Q = AS*g2Q;
			    matrix_multiply(3,3,3,AS,g2Q,ASNg2Q); 
			    //ASNg2Q = A + ASNg2Q;
			    matrix_add(3,3,A,ASNg2Q,ASNg2Q);
			    //ASNCNC = AS*CNC;
			    matrix_multiply(3,3,3,AS,CNC,ASNCNC); 
			    //AE = ASNg2Q - ASNCNC;
			    matrix_sub(3,3,ASNg2Q,ASNCNC,AE);
			    //AExo =  AE*xo;
			    matrix_multiply(3,3,1,AE,xo_aux,AExo);
			    //Bu = B*u;
			    matrix_multiply(3,3,1,B,u_aux,Bu);
			    //xob = ASNCNC*y;
			    matrix_multiply(3,3,1,ASNCNC,y_aux,xob);
			    //xo = AExo + Bu; 
			    matrix_add(3,1,AExo,Bu,xoa);
			    //xo = xo + xob;
			    matrix_add(3,1,xoa,xob,xoa);
			    //xo = xo + Brr;
			    matrix_add(3,1,xoa,Brr,xo_aux);
			    //S = AS*A';
			    matrix_multiply(3,3,3,AS,AT,Sa); 
			    //S = S + DD;
			    matrix_add(3,3,Sa,DD,S);
            	 	}
            	       else{
            	       
            	       		//Sg2Q = S*g2Q;
			    	matrix_multiply(3,3,3,S,g2Q,Sg2Q); 
				//Sg2Q = I - Sg2Q;
				matrix_sub(3,3,I,Sg2Q,Sg2Q);
				//IS = Sg2Q^(-1);
				matrix_inv33(Sg2Q,IS);
				//AS = A*IS;
				matrix_multiply(3,3,3,A,IS,AS); 
				//ASxo = AS*xo;
				matrix_multiply(3,3,1,AS,xo_aux,ASxo);
				//Bu = B*u;
				matrix_multiply(3,3,1,B,u_aux,Bu);
				//xo = ASxo + Bu;
				matrix_add(3,1,ASxo,Bu,xoa);
				//xo = xo + Brr;
				matrix_add(3,1,xoa,Brr,xo_aux);
		
				//Si = S^(-1);
				matrix_inv33(S,Si);
				//Si = Si - g2Q;
				matrix_sub(3,3,Si,g2Q,Si);
				//Sg2Q = Si^(-1);
				matrix_inv33(Si,Sg2Q);
				//AS = A*Sg2Q;
				matrix_multiply(3,3,3,A,Sg2Q,AS); 
				//S = AS*A';
				matrix_multiply(3,3,3,AS,AT,Sa); 
				//S = S + DD;
				matrix_add(3,3,Sa,DD,S);        	       
            	       }
            	       // save S
            	       
            	   } 
		    	
            	     
            	             	      
		}
		// let the xo_aux be the newest xo
////		printf("P: xo_aux after fix\n");		
////		matrix_print(3,1,xo_aux);  
		memcpy(xo,xo_aux,sizeof(xo));
////		printf("P: xo after fix\n");		
////		matrix_print(3,1,xo);

	}
 	printf("P: done estimator + controller\n");
 	// reset rxflag and npkTestRPLC.nc:616: called object is not a function, command, event or task
    	rxflag = 0;
    	npkts_rx = 0;
    }

    // clear pktlog for the old position
    pktlog[MD_index] = 0;
    printfflush();
 
  }
  
  }
  
  event void Timer.fired(){

 post processData();
 
 }
 

event void SplitControl.stopDone(error_t err){}

/*********************************************************************
		* Sending function at the sensor
*********************************************************************/

  task void sendTask(){
    struct sockaddr_in6 dest;
	nx_int16_t temp[5];
   uint8_t i;

    for(i=0;i<10;i++){
      temp[i] = 0xABCD;
    }
    
    
    memcpy(dest.sin6_addr.s6_addr, call RPLRoute.getDodagId(), sizeof(struct in6_addr));

    if(dest.sin6_addr.s6_addr[15] != 0) // destination is set as root!
      ++counttx;


    dest.sin6_port = htons(UDP_PORT);
    call RPLUDP.sendto(&dest, temp, 20);
  }

/*********************************************************************
	 * Matrix functions
********************************************************************/
	
void matrix_transpose(uint8_t m, uint8_t p, nx_float matrix[m][p], nx_float output[p][m])
{
	uint8_t i, j;
	for (i = 0; i < m; i++)
		for (j = 0; j < p; j++)
			output[j][i] = matrix[i][j];
}

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

void matrix_eq(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float output[m][n])
{
    uint8_t i,j;
    for(i = 0; i < m ; i++)
    	for(j = 0; j < n; j++)
    		output[i][j] =  matrix1[i][j];
}  

void matrix_sub(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n])
{
    uint8_t i,j;
    for(i = 0; i < m ; i++)
    	for(j = 0; j < n; j++)
    		output[i][j] =  (matrix1[i][j] - matrix2[i][j]);
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
   
   for(j=0;j<3;j++){
      for(i=0;i<3;i++)
output[j][i] = ((a[(i+1)%3][(j+1)%3] * a[(i+2)%3][(j+2)%3]) - (a[(i+1)%3][(j+2)%3]*a[(i+2)%3][(j+1)%3]))/ determinant;
    }

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
	
	void printfFloatU(nx_float toBePrinted) {
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
		printf("U: %c%ld.%d%d%d \n", c, fi, (uint8_t) f0, (uint8_t) f1,
				(uint8_t) f2);
				
		
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
