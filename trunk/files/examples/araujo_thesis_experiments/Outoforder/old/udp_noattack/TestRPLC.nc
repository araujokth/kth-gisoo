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
//#include "color.h"

#include "blip_printf.h"
/**
 * Implementation of the RadioCountToLeds application. RadioCountToLeds 
 * maintains a 4Hz counter, broadcasting its value in an AM packet 
 * every time it gets updated. A RadioCountToLeds node that hears a counter 
 * displays the bottom three bits on its LEDs. This application is a useful 
 * test to show that basic AM communication and timers work.
 *
 * @author Philip Levis
 * @date   June 6 2005
 */

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


    //interface Lcd;
    //interface Draw;
  }
}
implementation {

#ifndef RPL_ROOT_ADDR
#define RPL_ROOT_ADDR 1
#endif

#define UDP_PORT 5678

  //uint8_t payload[10];
  //struct in6_addr dest;
  struct in6_addr MULTICAST_ADDR;

  bool locked;
  uint16_t counter = 0;
  
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
  }

 


  uint32_t countrx = 0;
  uint32_t counttx = 0;

  event void RPLUDP.recvfrom(struct sockaddr_in6 *from, void *payload, uint16_t len, struct ip6_metadata *meta){

    nx_uint16_t temp[5];
    memcpy(temp, (uint8_t*)payload, len);
    call Leds.led2Toggle();
    ++countrx;
    printf("PRINTPKT: RX of ID, vals [%d,%d,%d], delay: %lu, countrx %lu, counttx %ld \n", TOS_NODE_ID, temp[2], temp[3], temp[4], countrx-temp[1], countrx, temp[1]);
    printfflush();
  }
  
  event void SplitControl.startDone(error_t err){
    while( call RPLDAO.startDAO() != SUCCESS );
    
    if(TOS_NODE_ID != RPL_ROOT_ADDR){
      call Timer.startOneShot((call Random.rand16()%2)*1024U);
    }
  }

/* Initial timer */
  event void Timer.fired(){
    //call MilliTimer.startOneShot(PACKET_INTERVAL + (call Random.rand16() % 100));
    call MilliTimer.startOneShot(PACKET_INTERVAL);
  }

/* SEND TASK */
  task void sendTask(){
    struct sockaddr_in6 dest;

    nx_uint16_t temp[5];
	// temp[0] TOS_NODE_ID
	// temp[1] conttx
	// temp[2] x1
	// temp[3] x2
	// temp[4] xc

    uint8_t i;

    for(i=0;i<5;i++){
      temp[i] = 0;
    }

    temp[0] = TOS_NODE_ID;
    temp[1] = counttx;

    memcpy(dest.sin6_addr.s6_addr, call RPLRoute.getDodagId(), sizeof(struct in6_addr));

    if(dest.sin6_addr.s6_addr[15] != 0) // destination is set as root!
      ++counttx;

    call Leds.led0Toggle();

    dest.sin6_port = htons(UDP_PORT);

    printf("PRINTPKT: Generate Packet at %d, value [%d,%d,%d] and count %d \n", TOS_NODE_ID, temp[2], temp[3], temp[4], temp[1] );
    call RPLUDP.sendto(&dest, temp, 20);
  }

/* Periodic timer */
  event void MilliTimer.fired(){ 
	if(TOS_NODE_ID == 1){
		call Leds.led1Toggle();
		call MilliTimer.startOneShot(PACKET_INTERVAL);
    		post sendTask();
	}
  }

  event void SplitControl.stopDone(error_t err){}

}
