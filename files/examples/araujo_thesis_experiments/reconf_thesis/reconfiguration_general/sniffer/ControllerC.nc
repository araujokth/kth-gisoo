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
 * @author Jose Araujo <araujo@kth.se>
 * Modified by Behdad Aminian <behdad@kth.se>
 * @version  $Revision: 1.0 Date: 2011/12/3 $ 
 */



#include "app_parameters.h"
#include "app_wt_calibration.h"
#include <printf.h>

module ControllerC {
	uses {

	/*********************************************************************
		* 1) Interfaces definition
	*********************************************************************/

		interface Boot;
		interface Leds;

	
		interface Receive;
		interface Packet;
		interface AMPacket;
		interface AMSend;
		interface SplitControl as AMControl;
		
		interface GeneralIO as ADC0;
		interface GeneralIO as ADC1;
		interface Random;

	}
}
implementation {


#define maxv 20
//#define PRINTFENABLED 1

	/*********************************************************************
		* 2) Variable definition
	*********************************************************************/
	
	uint16_t u;	// control input
	uint16_t u0;	// control input in the end of 1st phase
	
	uint16_t kp;	// proportional gain
	uint16_t ki;	// integral gain	
	uint16_t kd;	// derivative gain

	uint16_t pc;	// proportional part of u
	uint16_t ic;	// integral part of u	
	uint16_t dc;	// derivative part of u
	
	nx_float x_ref;
	nx_float op_point;	// operating point for 1st phase
	nx_float outf;	// auxiliary variable
	uint16_t e;	// output error

	
	uint16_t beta; //  1v in the pump is approx. 273 units in the DAC
	
	// Other variables
	bool busy;
	bool transmitted;
	message_t pktToActuator;
	message_t pktToRelayNode;
	message_t pktBroadcast;

	nx_float x_int;
	
	nx_float K[3];
	uint16_t i_p;

	uint8_t m_state;
	uint8_t srcid;

	uint8_t flagphase;
	bool relay;
	bool start;

	uint32_t period;

	uint8_t ngbrnum0[maxv];
	uint8_t ngbrnn0[maxv];
	nx_float ngbrvalmu0[maxv];
	nx_float ngbrvaleta0[maxv];
	nx_float ngbrvaleta1[maxv];
			
	uint8_t ngbrnumrx[maxv];
	uint8_t ngbrnnrx[maxv];
	nx_float ngbrvalmurx[maxv];
	nx_float ngbrvaleta0rx[maxv];
	nx_float ngbrvaleta1rx[maxv];
	
	uint8_t faultnode[maxv];
		
	nx_float mmatchval;
	
	uint8_t ngbrct0;
	uint8_t ngbrct;

	uint8_t numpktC;
	uint16_t numpktCcount;
	uint16_t logpktC;
	uint16_t tmp;
	
	uint8_t counterrx;
	
	uint8_t trigger;
	
	// stuff for dist algorithm
	
	nx_float I[2][2];
	nx_float M[4];
	nx_float delta;
	nx_float alpha[8];
	nx_float Hc[4][8];
	nx_float wc[4][1];
	
	/*********************************************************************
		* 3) Booting functions and Variable value assignment
	*********************************************************************/

void matrix_multiply(uint8_t m, uint8_t p, uint8_t n, nx_float matrix1[m][p], nx_float matrix2[p][n], nx_float output[m][n]);
void matrix_multiply_scalar(uint8_t m, uint8_t n, nx_float scalar, nx_float matrix2[m][n], nx_float output[m][n]);
void matrix_add(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n]);
void matrix_sub(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n]);
void printfFloat(nx_float toBePrinted); // For printing out float numbers
void matrix_print( uint8_t m, uint8_t n, nx_float matrix[m][n]);

	event void Boot.booted() {
		
		uint8_t i;
		uint8_t ptrv;
				
		
		u = 0;
		x_int = 700;
		
		busy = FALSE;
		transmitted = FALSE;
		
		
		
		ngbrct0 = 0;
		ngbrct = 0;
		numpktC = 0;
		    
		//initialize the stats
		for(i=0;i<maxv;i++){
      			ngbrnum0[i] = 0;
			ngbrnn0[i] = 0;
			ngbrvalmu0[i] = 0;
			ngbrvaleta0[i] = 0;
			ngbrvaleta1[i] = 0;
			faultnode[i] = 0; 
    		}
    atomic {
			ADC12CTL0 = REF2_5V +REFON;
			DAC12_0CTL = DAC12IR + DAC12AMP_5 + DAC12ENC;
		}
					
		
		//To not disturb the Sensor Values
		call ADC0.makeInput();
		call ADC1.makeInput();
		DAC12_0DAT = 0;

		flagphase = 1;
		call AMControl.start(); // We need to start the radio
		
		counterrx = 0;
		
		// distributed algorithm
		
		I[0][0] = 1.0000;
		I[0][1] = 0.0000;
		I[1][0] = 0.0000;
		I[1][1] = 1.0000;

		M[0] = 1.0000;
		M[1] = 0.6000;
		M[2] = 0.7000;
		M[3] = 1.1000;
		
		
		alpha[0] = 0.5903;
		alpha[1] = -0.0181;
		alpha[2] = 0.3542;
		alpha[3] = -0.0108;
		alpha[4] = 0.4132;
		alpha[5] = -0.0127;
		alpha[6] = 0.6494;
		alpha[7] = -0.0199;
		
		Hc[0][0] = 0.0278;
Hc[0][1] = 0.0000;
Hc[0][2] = 0.0167;
Hc[0][3] = 0.0000;
Hc[0][4] = 0.0194;
Hc[0][5] = 0.0000;
Hc[0][6] = 0.0306;
Hc[0][7] = 0.0000;
Hc[1][0] = 0.0000;
Hc[1][1] = 0.0000;
Hc[1][2] = 0.0000;
Hc[1][3] = 0.0000;
Hc[1][4] = 0.0000;
Hc[1][5] = 0.0000;
Hc[1][6] = 0.0000;
Hc[1][7] = 0.0000;
Hc[2][0] = 0.0000;
Hc[2][1] = 0.0278;
Hc[2][2] = 0.0000;
Hc[2][3] = 0.0167;
Hc[2][4] = 0.0000;
Hc[2][5] = 0.0194;
Hc[2][6] = 0.0000;
Hc[2][7] = 0.0306;
Hc[3][0] = 0.0000;
Hc[3][1] = 0.0000;
Hc[3][2] = 0.0000;
Hc[3][3] = 0.0000;
Hc[3][4] = 0.0000;
Hc[3][5] = 0.0000;
Hc[3][6] = 0.0000;
Hc[3][7] = 0.0000;

wc[0][0] = 0.0502;
wc[1][0] = 0.0000;
wc[2][0] = -0.0015;
wc[3][0] = 0.0000;

		delta = -0.02000;
		
		numpktCcount = 1;	
		tmp = 1;
	}


	/********************************************************************
		* Message reception 
	*********************************************************************/

	event message_t* Receive.receive(message_t* msg, void* payload, uint8_t len) {

		uint8_t i;
		uint8_t pi;
		uint8_t pf;
		nx_float eta[8][1];
		nx_float mm[4][1];
		nx_float resa[4][1];
		nx_float resb[1][4];
		nx_float resc[1][1];
		nx_float resd[4][1];
		nx_float rese[1][1];
		uint16_t mmvalsend;
		// If it is a msg from the controller with the sensor values
		srcid = call AMPacket.source(msg);
		
		#ifdef PRINTFENABLED
			printf("P: srcid %d\n",srcid);		
		#endif
		if ((len == sizeof(xpkt)) && (srcid == 1)) {
						
			call Leds.led2Toggle();  //toggle led 2 whenever receiving a message
			
			pi = 0;
			pf = 1;
			for(i=0;i<4;i++){
				eta[pi][0] = ngbrvaleta0rx[i]; 
				eta[pf][0] = ngbrvaleta1rx[i];
				pi = pi + 2;
				pf = pf + 2;
			}
			#ifdef PRINTFENABLED
				printf("P: print eta\n");
				printfFloat(eta[0][0]);
				printfFloat(eta[1][0]);
				printfFloat(eta[2][0]);
				printfFloat(eta[3][0]);
				printfFloat(eta[4][0]);
				printfFloat(eta[5][0]);
				printfFloat(eta[6][0]);
				printfFloat(eta[7][0]);
			// calc statistics
			#endif
			matrix_multiply(4,8,1,Hc,eta,mm);
			#ifdef PRINTFENABLED
				printf("P: print mm\n");
				printfFloat(mm[0][0]);
				printfFloat(mm[1][0]);
				printfFloat(mm[2][0]);
				printfFloat(mm[3][0]);
			#endif
			matrix_sub(4,1,mm,wc,resa);
			#ifdef PRINTFENABLED
			printf("P: print resa\n");
			#endif
			resb[0][0] = resa[0][0];
			resb[0][1] = resa[1][0];
			resb[0][2] = resa[2][0];
			resb[0][3] = resa[3][0];
			resd[0][0] = wc[0][0];
			resd[0][1] = wc[1][0];
			resd[0][2] = wc[2][0];
			resd[0][3] = wc[3][0];
			matrix_multiply(1,4,1,resb,resa,resc);
			matrix_multiply(1,4,1,resd,wc,rese);
			#ifdef PRINTFENABLED
				printf("P: print resc\n");
				printfFloat(resc[0][0]);
				printf("P: print resc\n");
				printfFloat(rese[0][0]);
			#endif
			// send the values to serial port
			outf = 100*resc[0][0]/rese[0][0];
			#ifdef PRINTFENABLED
				printf("P: print outf\n");
				printfFloat(outf);
			#endif
			if(outf < 0) outf = 0;
			else if(outf > 4095) outf = 4095;
			mmvalsend = (uint16_t) outf;
			#ifdef PRINTFENABLED
				printf("P: print valtosend %d\n",mmvalsend);
			#endif
			DAC12_0DAT = mmvalsend;	
			
			// clear the rx buffer	
			counterrx = 0;
			for(i=0;i<maxv;i++){
				ngbrnumrx[i] = 0;
				ngbrnnrx[i] = 0;
				ngbrvalmurx[i] = 0;
				ngbrvaleta0rx[i] = 0;
				ngbrvaleta1rx[i] = 0;
    			}
    			// clear eta	
    			
    			pi = 0;
			pf = 1;
			for(i=0;i<4;i++){
				eta[pi][0] = 0; 
				eta[pf][0] = 0;
				pi = pi + 2;
				pf = pf + 2;
			}		
		
		}	
		
		// receiving the beacon with stats	
		if ((len == sizeof(beaconpkt)) && (srcid != 1) ) {		
			beaconpkt* apkt = (beaconpkt*)payload;
			
			uint8_t ngbrnumtmp;
			uint8_t ngbrnntmp;
			nx_float ngbrvalmutmp;
			nx_float ngbrvaleta0tmp;
			nx_float ngbrvaleta1tmp;

				
			ngbrnumtmp = srcid;
			ngbrnntmp = (apkt->beacon_nn);
			ngbrvalmutmp = (nx_float) (apkt->beacon_mu)/1000;
			
			ngbrvaleta0tmp = (nx_float) (apkt->beacon_eta0)/1000;
			ngbrvaleta1tmp = (nx_float) (apkt->beacon_eta1)/1000;
			#ifdef PRINTFENABLED
				printf("P: print eta received from neighbor %d\n", srcid);
				printfFloat(ngbrvaleta0tmp);
				printfFloat(ngbrvaleta1tmp);
				printf("P: print mu received from neighbor %d\n", srcid);
				printfFloat(ngbrvalmutmp);
			#endif

			ngbrnumrx[srcid-2] = ngbrnumtmp;
			ngbrnnrx[srcid-2] = ngbrnntmp;
			ngbrvalmurx[srcid-2] = ngbrvalmutmp;
			ngbrvaleta0rx[srcid-2] = ngbrvaleta0tmp;
			ngbrvaleta1rx[srcid-2] = ngbrvaleta1tmp;
			counterrx = counterrx + 1;
			
			call Leds.led0Toggle();  //toggle led 0 whenever receiving a message of this type
				
		}	
			
		return msg;
	}

	

	/*********************************************************************
	 	* 9) Message functions
	 **********************************************************************/

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
		if ((&pktToActuator == msg)) {
			call Leds.led1Toggle();
			busy = FALSE;
		}
	}

/*********************************************************************
	 	* Matrix stuff
	 **********************************************************************/
	

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

void matrix_sub(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n])
{
    uint8_t i,j;
    for(i = 0; i < m ; i++)
    	for(j = 0; j < n; j++)
    		output[i][j] =  (matrix1[i][j] - matrix2[i][j]);
}  


void matrix_multiply_scalar(uint8_t m, uint8_t n, nx_float scalar, nx_float matrix2[m][n], nx_float output[m][n])
{
    uint8_t i, j; 
    for (i = 0; i < m; i++)
        for (j = 0; j < n; j++)
            output[i][j] = 0.0;
            
    for (i = 0; i < m; i++){
        for (j = 0; j < n; j++){
            output[i][j] += scalar*matrix2[i][j];}}
        	

}

void matrix_add(uint8_t m, uint8_t n, nx_float matrix1[m][n], nx_float matrix2[m][n], nx_float output[m][n])
{
    uint8_t i,j;
    for(i = 0; i < m ; i++)
    	for(j = 0; j < n; j++)
    		output[i][j] =  (matrix1[i][j] + matrix2[i][j]);
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
	
	/*********************************************************************
	 	* END *
	 **********************************************************************/

}
