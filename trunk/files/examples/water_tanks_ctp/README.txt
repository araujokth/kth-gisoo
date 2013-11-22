README for Water_Tanks_CTP
Author/Contact: behdad@kth.se

Description:

All the mote communicate on channel 15.
The communication between sensors, relays and, Controller is routed by CTP.
The communication between controller and actuators is direct(without CTP) and controller requests the ack for its sent packets. 
Whenever the controller receive the sensor data from a specific sensor, it will calculate the actuation value an send it to the actuator with the mote-id = sensor_id +10.

Tools:

Known bugs/limitations:

None.

