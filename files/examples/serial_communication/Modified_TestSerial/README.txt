README for ModifiedTestSerial
Author/Contact(For Original TestSerial application): tinyos-help@millennium.berkeley.edu
Modified by: behdad@kth.se
Description:

ModifiedTestSerial is a simple application that may be used to test that the
Simulink and Cooja in GISOO can communicate over the serial
port. Mote application receive a counter packets with size of 16 bytes over the serial port. When the mote application receives a counter packet, it displays the bottom three bits on its
LEDs. (This application is similar to RadioCountToLeds, except that it
operates over the serial port.) Likewise, the mote also sends packets
to the serial port at 1Hz which can be received by Simulink in the GISOO environment.


Tools:

Known bugs/limitations:

None.

