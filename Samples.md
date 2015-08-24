

---

# Examples tutorial #
This section provides explanations about different example scenarios. The scenarios have been selected so that they cover different areas. At the beginning it has been tried to show how the ADC and DAC can be used in GISOO and in the next step the required details for serial communication has been explained. The usage of serial communication also has been shown in a simple example. In the next section the previous  [simple scenario ](https://code.google.com/p/kth-gisoo/wiki/How_to_use_it#Simple_Scenario) (water tanks scenario) has been modified to  provide use of serial communications in its scenario. In the next section the water-tanks scenario has been extended to show the usage of CTP routing protocol in the routing layer.
In overall this section provides examples from very simple and basic aspects until advanced ones and can be considered as tutorial for GISOO.
## Basic Examples ##
In all the examples we are using the TinyOS to provide the mote programs, but in order to show the adaptability of the GISOO with the Contiki codes, we provided the first example in two sections using the both Contiki and TinyOS codes.
### ADC data transmission in GISOO ###
In order to show the ADC data communication in GISOO we create a scenario that mote application in Cooja will sense and ADC value which has been generated in Simulink.
#### ADC data transmission in GISOO - Using TinyOS ####
TinyOS codes and Simulink Model can be reached: [here](https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fadc_data_transmission)

This example, explains the ADC communication procedure in GISOO. For this example you can download provided TinyOS code which should be used to create the Cooja simulation. Also the designed Simulink model for this scenario is available for download. In the first step you have to compile the TinyOS code by the following command:(To use this command  the TinyOS should be installed in advanced.)
```
make telosb install
```
More explanation about TinyOS and its commands can be found in the [TinyOS tutorials](http://tinyos.stanford.edu/tinyos-wiki/index.php/TinyOS_Tutorials). This code will read the ADC values of ADC0 and ADC1 in the mote every 200 ms and whenever a ADC data is received, the three least significant bits in the value  of ADC0 are displayed by the motes' LEDs.
After compiling the ADCReader code, you should run a new simulation in Cooja and create a new mote in that simulation environment by using the main.exe file of ADCReader which will be created in the build folder after its compilation. More details about adding mote in Cooja has been explained [here](https://code.google.com/p/kth-gisoo/wiki/How_to_use_it#Adding_motes_to_the_simulation). Finally you should activated the "GISOO-Simulink plugin" for the created mote.
In the next step you have to download the Simulink model and then GISOO simulation is ready. This Simulink model will generate different values by using the sin function, to be read as the ADC value bye the mote. The following figure shows the simulation environment in the Cooja and Simulink.

<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/adcreader/ADCReader.png'>
<p>

As it has been shown in the previous picture after running the simulation the Leds will blink according to the data which it could be read in the ADC0.<br>
The following picture shows inside of the "GISOO-Simulation Environment" in the Simulink model, which contains one mote with two incomes, one for ADC0 and another one for ADC1.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/adcreader/ADCReader_CoojaPlatform.png'>
<p>

<h4>ADC data transmission in GISOO - Using Contiki</h4>
Contiki codes and Simulink Model can be reached: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fadc_data_transmission_contiki%253Fstate%253Dclosed'>here</a>




In order to provide an example for the ADC communication in Contiki we used the example which were available in <a href='http://stackoverflow.com/questions/21047965/how-to-read-temperature-humidity-and-light-measures-with-contiki-os'>this page</a>.<br>
Since this code tries to read ADC data from ADC4 instead of ADC0 or ADC1 which has been used in the other examples, we modified the mote block in Simulink library to activate ADC4 as well. You can see the mote block modification in the following picture.<br>
<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/adcreader/ADC4.png'>
<p>

For using the Contiki code in COOJA, you have to brows and insert the C code and compile it in Cooja or you can use the provided "Onlinelight.sky" which is the result of compilation.<br>
the final simulation scenario should be like this:<br>
<br>
<h3>DAC data transmission in GISOO</h3>
TinyOS codes and Simulink Model can be reached: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fdac_data_transmission'>here</a>

This example shows a simple DAC communication and similar to the previous example, you can download the provided TinyOS code and Simulink model for this example. The provided TinyOS code (DACWriter) will write a counter value on the DAC0 of the mote and whenever the DAC value is written the Led2 will toggle. Following picture shows the Simulink and Cooja environment together.<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/dacwriter/DACWriter.png'>
<p>

As the previous picture shows the mote generate the counter values and send it over the DAC0. For each DAC value the led2 toggled. On the other side the Simulink will receive this value and show it in the scope.<br>
The following picture shows the amount of the received counter values in the Simulink on the right side and inside of the "GISOO-Simulation Environment" on the left side.<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/dacwriter/DACWriter_CoojaPlatform.png'>
<p>



<h3>Serial Communication</h3>
More over to the ADC and DAC, GISOO also support the serial Communication between Simulink and Cooja. The designed procedure in GISOO can provide two types of serial communication in its scenarios.<br>
<br>
<ol><li>Mote in Cooja send data via serial port to the Simulink, without expecting to receive data from Serial port.<br>
</li><li>Mote In Cooja send data via serial to the Simulink and it expects to receive a reply serial data from Simulink after a specific amount of time.</li></ol>

The next two examples explain each of these methods in details.<br>
<br>
<h4>One Way Serial Communication</h4>
TinyOS codes and Simulink Model can be reached: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fserial_communication'>here</a>

Like all the scenarios in GISOO, this example also contains two parts, TinyOS code and the Simulink model. The provided TinyOS code for this example is a modified version of standard "testserial" code which has located in the  "tinyos-2.x/apps/tests/TestSerial" or you also can download it from <a href='http://www.tinyos.net/tinyos-2.1.0/apps/tests/TestSerial/'>here</a>.<br>
The original TestSerial application sends a counter value on the serial port every 1 second and also it shows the last three significant bits of the received serial data by leds. This application contain a java program which also generate a counter value on PC and send it to the mote's serial port.<br>
To use this application in GISOO as an example for serial communication, we need to modify it and understanding these modification required to understand the message structure for the serial communication in GISOO which will be explained in below:<br>
<br>
<h5>Message structure for the serial communication in GISOO</h5>
The details of designed messages structure for data transmission between Simulink and Cooja in GISOO has been explained in <a href='https://code.google.com/p/kth-gisoo/wiki/technical_details#Data_Communication'>Data Communication</a>. But in very short, to understand this part, we should know that, GISOO has been designed so that for serial communication between Cooja and Simulink the serial data should have <b>16 bytes structure</b> (The data structure that you define in TinyOS code of your mote to be sent or received via serial should have 16 bytes). It means that the serial data can not be longer than 16 byes (if you need to communicate more than 16 bytes you should divide it to different messages) or also if your message is smaller than 16 byte you still have to define 16 byte data structure but you can leave the unneeded parts, empty.<br>
<br>
For this example in order to adapt the "testserial" application with the required message structure, we changed the data structure of the counter message to make it in a 16 bytes structure. In the defined code for this scenario, in the "TestSerial.h" the serial message data contains 16 bytes as it has been shown in the following:<br>
<br>
<pre><code>typedef nx_struct test_serial_msg {<br>
    nx_uint16_t counter;<br>
    nx_uint16_t counter2[7];   //Added to the original structure to convert it to a 16 bytes structure.<br>
} test_serial_msg_t;<br>
</code></pre>

We also removed the java application which were not needed in this scenario and modified the make-file.<br>
The following figure shows the implemented scenario in GISOO. In this scenario the virtual mote in Cooja will generate a counter value periodically and send it to its mirror mote in Simulink. Then the mirror mote extract the counter value from the received serial message and show it in a scope.<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SerialComm_OneWay.png'>
<p>
Following picture shows inside of the "GISOO-Simulation Environment" block in left and the scope of counter valued in right.<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SerialComm_OneWay_Result.png'>
<p>

<h4>Two Ways Serial Communication</h4>
TinyOS codes and Simulink Model can be reached: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fserial_communication'>here</a>

As it has been mentioned, another supported type of the serial communication in GISOO, is two ways communication. In this method first the virtual mote in Cooja will send a serial message to its mirror mote in Simulink and will request for receiving a serial message as reply after specific amount of time. This example explains the required details for this type of serial communication. The TinyOS code that we used for this example is the the "testSerial" application, the same as we used in the previous example. But in this example the serial messages will pass a circle from Cooja to Simulink and then from Simulink back to Cooja. In this Scenario the virtual mote in Cooja first will send a counter value via the serial data to its mirror mote in Simulink but it also will request to receive a serial message in 1 ms after the sending the message. On the other side in the Simulink the mirror mote will receive the serial message and extract the counter value, but again will send the received counter value via another serial message to the mote in Cooja; and in the result the virtual mote in Cooja will show the last three bits of the received counter value by its Leds.<br>
But in order to receive a serial message back, in Cooja we should select the provided check-box in the GISOO plug-in which will specify the require of receiving the serial message after a specific amount of time which should be mentioned in provided text-box. The general view of the simulated scenario and mentioned check-box in the GISOO-Simulink plugin has been shown in the following figure. It also shows the mote's leds which blink in result of receiving a serial message back from Simulink.<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SerialComm_TwoWays2.png'>
<p>

The following picture shows the inside of the "GISOO-Simulation Environment" block in the Simulink model. As you can see the counter value from received serial message will be sent back via the serial port to the mote in Cooja. But there are two important points in this model which worth to be mentioned.<br>
The first point is about the location of counter value in the received serial message. Since in the defined message structure in the testserial.h the first two bytes belongs to the counter value and the rest of the 16 bytes are unused. So when we receive the the serial message in Simulink, the counter value should be rebuild from the first two bytes and also we should put it in the proper location (same location in this scenario) when we want to send it back to mote. You can see the testSerial.h in the following:<br>
<br>
<pre><code><br>
#ifndef TEST_SERIAL_H<br>
#define TEST_SERIAL_H<br>
<br>
typedef nx_struct test_serial_msg {<br>
nx_uint16_t counter;<br>
nx_uint16_t counter2[7];<br>
<br>
} test_serial_msg_t;<br>
<br>
enum {<br>
  AM_TEST_SERIAL_MSG = 6, <br>
};<br>
<br>
#endif<br>
</code></pre>

The other important point, is about the "AM_TEST_SERIAL_MSG" value which is 6 in this sample. This value has been used in the "ModifiedTestSerialAppC" as it has been shown in the following:<br>
<br>
<pre><code>components SerialActiveMessageC as AM;<br>
.<br>
.<br>
.<br>
 App.Receive -&gt; AM.Receive[AM_TEST_SERIAL_MSG];<br>
 App.AMSend -&gt; AM.AMSend[AM_TEST_SERIAL_MSG];<br>
</code></pre>
This Value will act similar as the port number for the serial message and specify this message belongs to which application. This point is important when we want to send a serial message from Simulink to Cooja because this value should be the same with its corresponding value in the generated serial message in Simulink, otherwise the generated message will be discard in the mote. So for this aim in the mote parameter in Simulink, there is a value of "Serial Packet Type" which should be set with the same value of AM_TEST_SERIAL_MSG in this sample (6 in this scenario).<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SerialComm_TwoWays_CoojaPlatform.png'>
<p>


<h4>How to add serial communication in your TinyOS code</h4>
This section is not completely related to hte GISOO and how it works, but it explains how to add the serial communication in part of your own TinyOS code.<br>
<br>
For adding serial communication to your code, you need to pass the following steps:<br>
<ul><li>Creating a header file including your message structure and defining the "Active message id"<br>
</li><li>Introduce the required components in your configuration file.<br>
</li><li>Add the required interfaces in the module file.<br>
</li><li>Modify the desired place of your code to send serial data.</li></ul>

To add serial communication to your application in the first step you should create your message structure. In this section we selected the Blink application which is one of the basic examples in TinyOS and you can find it at "tinyos-2.x/apps" or download it from <a href='http://www.tinyos.net/tinyos-1.x/apps/Blink/'>here</a>. In this section we want to modify the application so that make it very similar to the testSerial application and send a counter value to the serial port whenever its first led toggled. We use the same message structure as it has been used in testSerialApplication which has been shown in the previous section, but again you can see it in the following picture (You should consider that the message structure should have 16 bytes even you want to communicate less amount of data):<br>
<br>
<pre><code><br>
#ifndef TEST_SERIAL_H<br>
#define TEST_SERIAL_H<br>
<br>
typedef nx_struct test_serial_msg {<br>
nx_uint16_t counter; // This is the place for our actual counter value <br>
nx_uint16_t counter2[7]; // This has been added to convert the message structure to 16 bytes<br>
<br>
} test_serial_msg_t;<br>
<br>
enum {<br>
  AM_TEST_SERIAL_MSG = 6, <br>
};<br>
<br>
#endif<br>
</code></pre>


You need to copy the shown code in a file named "TestSerial.h" which is located in the application folder.<br>
<br>
In the second step we need to add necessary components in the configuration file. these component has been shown in the following part:<br>
<br>
<pre><code><br>
// Added parts for serial communication:<br>
  components SerialActiveMessageC as AM;<br>
  BlinkC.Control -&gt; AM;<br>
  BlinkC.AMSend -&gt; AM.AMSend[AM_TEST_SERIAL_MSG];<br>
  BlinkC.Packet -&gt; AM;<br>
<br>
</code></pre>


Now you should wire some new components in your application. The following part shows this wiring section:<br>
<br>
<pre><code><br>
//This part should be added at the beginning of your code<br>
  uses interface Packet;<br>
  uses interface AMSend;<br>
  uses interface SplitControl as Control;<br>
<br>
</code></pre>


You should also declare these variable at the beginning of the implementation part:<br>
<pre><code><br>
// Required variables for serial communication<br>
  uint16_t counter = 0; // The counter value<br>
  message_t packet; // The serial message <br>
  bool locked = FALSE; // Flag for checking if serial is busy or not<br>
<br>
<br>
</code></pre>

you should also start control component in the boot section:<br>
<br>
<pre><code><br>
 call Control.start();<br>
<br>
</code></pre>



Finally you should add the following part in the place that you want to send the serial data<br>
<br>
<pre><code><br>
 if (locked) {<br>
      return;<br>
    }<br>
    else {<br>
      test_serial_msg_t* rcm = (test_serial_msg_t*)call Packet.getPayload(&amp;packet, sizeof(test_serial_msg_t));<br>
      if (rcm == NULL) {return;}<br>
      if (call Packet.maxPayloadLength() &lt; sizeof(test_serial_msg_t)) {<br>
	return;<br>
      }<br>
<br>
      rcm-&gt;counter = counter;<br>
      if (call AMSend.send(AM_BROADCAST_ADDR, &amp;packet, sizeof(test_serial_msg_t)) == SUCCESS) {<br>
	locked = TRUE;<br>
      }<br>
    }<br>
<br>
 counter++;<br>
<br>
</code></pre>


And these event should also be added to the application to make it complete:<br>
<br>
<pre><code><br>
 event void AMSend.sendDone(message_t* bufPtr, error_t error) {<br>
    if (&amp;packet == bufPtr) {<br>
      locked = FALSE;<br>
    }<br>
  }<br>
<br>
<br>
<br>
event void Control.startDone(error_t err) {<br>
    if (err == SUCCESS) {<br>
      call Timer0.startPeriodic( 250 );<br>
    }<br>
  }<br>
  event void Control.stopDone(error_t err) {}<br>
<br>
</code></pre>


You can find the complete modified application with explanation comments <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fserial_communication%2FAddingSerialCommToBlink%253Fstate%253Dclosed'>here</a>

<h2>Advanced Examples</h2>
Most of the real scenarios are not only a simple sensor or actuator but they are a mixture of those. For this reason their simulation environment consist of ADC reading and DAC writing and even maybe serial communication together and in this part some of these mixed scenarios will be explained.<br>
<br>
One of the simplest mixed scenario is only made of combination of ADC and DAC. The schematic of this environment has been shown in the following picture.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/advancescenarios/ADCDACComm.png' height='250px' width='250px'>
<p>
In these types of scenarios the sensor in Cooja will sense the required sensing data from the plant model in Simulink and after passing some relays and doing some controlling calculation in some other motes, the actuation data will be sent via DAC communication to the plant model. The above model, can also be considered as the model of simulation environment in Simulink which shows the relation between mote blocks in the "GISOO-Simulation Environment" and the plant model. One example of this type of scenarios is the Water-tanks scenario with the controller in one mote which has been explained as a "<a href='https://code.google.com/p/kth-gisoo/wiki/How_to_use_it#Simple_Scenario'>Simple Scenario</a>" in "<a href='https://code.google.com/p/kth-gisoo/wiki/How_to_use_it'>How to Use GISOO</a>".<br>
<br>
The other type of scenario is which we locate the Controller inside a PC instead of one mote (in the need of powerful computation resource). Sensor will sense the sensing data from plant model and pass it to a Base-Station mote. Base-Station will send the data to the PC (for instance a controller model in Simulink) Which has shown as CPU block. Finally this CPU block will calculate the actuation value and send it directly to the plant model.<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/advancescenarios/SerialType1.png' height='250px' width='250px'>
<p>

The other type of scenarios, is similar to the previous one and the base station will send the sensor data to the CPU block in the PC (or Simulink controller model) but in these scenarios the CPU block will send back the calculated actuation data as a reply to the Base-Station and Base-Station will forward it to the actuator mote inside the Cooja. Then the actuator will send the actuation value via a DAC message to the plant model in the Simulink.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/advancescenarios/SerialType2.png' height='250px' width='250px'>
<p>


This type of scenario is useful to simulate those systems with the need of powerful processing resource as a controller and also exist of actuation motes. In this situation the "Base-Station" application will act as a gateway that can be used to transfer our collected data to a PC where the controlling computation will be done and then sending back the results to the wireless system in Cooja.<br>
To show the usage of this model we modified the water tank scenario so that the controller will be located in a PC instead of one mote (As it was in the simple example). The new controller is a Simulink model and will receive the serial data from base station and send back the actuation data to the Cooja. The details about this system had been explained in the following section.<br>
<br>
<h3>Double-tank system with the controller in PC</h3>
TinyOS codes and Simulink Model can be reached: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_controller_in_pc'>here</a>

In the section "<a href='https://code.google.com/p/kth-gisoo/wiki/How_to_use_it'>,How to use GISOO</a>" we introduced the double-tank system as a "<a href='https://code.google.com/p/kth-gisoo/wiki/How_to_use_it#Simple_Scenario'>,simple scenario</a>". In this section we explain the same simulation scenario but by locating the control process on a PC instead of a mote. The following picture shows both scenarios with controller in a mote or in a PC.<br>
<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SimpleScenarioMix.png'>
<p>

In order to simulate the scenario with the controller in PC first we should create all of our motes  in Cooja. so four motes should be created: one Sensor, one relay, one Base-Station and, one actuator. You can download the TinyOS code for the motes and Simulink Model from: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_controller_in_pc'>here</a>. These codes are exactly the same code as those we had for our simple scenario with the new message structure for the sensor and actuator and also instead of a code for controller we have a modified code for base-station which send the radio messages to the actuator instead of broadcast which will be explained in the following.<br>
<br>
As we had explained in "<a href='https://code.google.com/p/kth-gisoo/wiki/Samples#Message_structure_for_the_serial_communication_in_GISOO'>Message structure for the serial communication in GISOO</a>" the message structure for sending via serial port should have 16 bytes. For this reason we changed the data struct of the message from sensor to the controller (relay will just forward the message without any modification) to make in a 16 bytes structure. In the defined code for this scenario in the "app_parameters.h" the sensor data contains 16 bytes as it has shown in the following:<br>
<br>
<pre><code>// Type of message send by the sensor node to the relay node<br>
typedef struct SensorValues {<br>
	nx_uint16_t tankLevel[8]; // tankLevel[0] for the tank-1 and tankLevel[1] for tank-2 and the rest of the structure will not be used.<br>
} SensorValues;<br>
</code></pre>

For the same reason the message that will be sent from base-station to the actuator also has 16 bytes structure so that the structure of the message that should be received by actuator should also be changed to 16 byte:<br>
<br>
<pre><code>// Type of message send by the controller (base-station) to the actuator node<br>
typedef nx_struct EncMsg2SensorsAct {<br>
	nx_uint16_t u[8]; // Actuation value send to the actuator(from base-station). u[0] contains the actuation value and the rest of the data structure is empty.<br>
}EncMsg2SensorsAct;<br>
</code></pre>

The other point that has been mentioned about the codes in this scenario was the modified base-station. Since we would like to forward all the actuation values from PC to the actuator (not the other motes) we modified the send function of the base-Station so that it just forward the radio messages to the actuator (mote with id:4 in this scenario). As it has been shown in the following code, the "addr" value has been changed to a fix id of actuator mote (4).<br>
<pre><code>task void radioSendTask() {<br>
    uint8_t len;<br>
    am_id_t id;<br>
    am_addr_t addr,source;<br>
    message_t* msg;<br>
    <br>
    atomic<br>
      if (radioIn == radioOut &amp;&amp; !radioFull)<br>
	{<br>
	  radioBusy = FALSE;<br>
	  return;<br>
	}<br>
<br>
    msg = radioQueue[radioOut];<br>
    len = call UartPacket.payloadLength(msg);<br>
    addr = 5;//call UartAMPacket.destination(msg);<br>
    source = call UartAMPacket.source(msg);<br>
    id = call UartAMPacket.type(msg);<br>
<br>
    call RadioPacket.clear(msg);<br>
    call RadioAMPacket.setSource(msg, source);<br>
    <br>
    if (call RadioSend.send[id](addr, msg, len) == SUCCESS)<br>
      call Leds.led0Toggle();<br>
    else<br>
      {<br>
	failBlink();<br>
	post radioSendTask();<br>
      }<br>
  }<br>
</code></pre>

After preparing our TinyOS codes we have to create all the motes in Cooja. It has bees shown in the following picture. In this scenario mote-ids has been defined in the "app_parameters.h":<br>
<pre><code>        SENSOR_ADDRESS          =       1,<br>
        RELAYNODE_ADDRESS       =       2,<br>
        COORDINATOR_ADDRESS     =       3,<br>
        ACTUATOR_ADDRESS        =       4     <br>
</code></pre>

So we have to create the motes with the mentioned id but instead of Coordinator (Controller) we have to create our base-station. After creating, for those motes which needs to be connected to the plant or PC(for controlling purposes) we have to activate the "GISOO-Simulink plugin". In this scenario sensor needs to sense the water level from the plant, actuator should be able to send the actuation value to the plant, and also base-Station should be able to transfer the serial data to the controller model in the PC and receive the results back from that model. As it has shown the "GISOO-Simulink plugin" has been activated for these motes.<br>
<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SerialCommunication.png'>
<p>

The important point about this scenario is the serial communication which will be performed by using the Base-Station. As it has been mentioned, GISOO supports two type of serial communication in its scenarios.<br>
<br>
<ol><li>Mote in Cooja send data via serial to the Simulink without expecting to receive data from Serial. (In these types of scenarios The "GISOO-Simulink plugin" will automatically forward the motes serial data to it mirror motes in Simulink)<br>
</li><li>Mote In Cooja send data via serial to the Simulink and it expect to receive a reply serial data from Simulink after a specific amount of time. (In these types of scenarios you have to specify the necessity of receiving the serial reply by selecting the check-box in the "GISOO-Simulink plugin" and also mentioning the specific amount of in the provided text-box.) As you can see in the above picture the check-box of the plug-in for the base-station has been ticked and it expect to receive the reply 1 millisecond (default value) after the serial data has been send via the serial to Simulink.</li></ol>

Like all other scenarios in GISOO the other side of simulation should be designed in Simulink. In the Simulink part, the location of plant and the "GISOO-Simulation Environment" are exactly the same as the simple scenario without serial communication, which has been shown in the following picture.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/SimulationModelForSerialCommunication.png' height='450px' width='650px'>

<p>

The main difference between this scenario and the simple scenario without serial communication is inside the "GISOO-Simulation Environment". As it has shown in the following picture, in this scenario we have to add another mote as base-Station to the Simulation Environment which receives the serial data from its mirror mote in Cooja and forward the data to controller model which will calculate the actuation value according to its input and finally it will send the calculated to the base-Station in order to forward the value to the actuator.<br>
<br>
Since in GISOO the serial data should have the fix 16 bytes data structure, the output serial data should be divided to it 16 bytes and then according to your defined data types, you can recreate your data from the serial message. In this scenario we send two values as a water levels that each of them has two bytes (the message structure of sensor has been explained previously in this page)<br>
<br>
<pre><code>nx_uint16_t tankLevel[8]; // tankLevel[0] for the tank-1 and  tankLevel[1] for tank-2 <br>
</code></pre>
So after receiving the serial data we have to recreate two water tank values and for this aim we can use the provided blocks in GISOO Simulink Library to generate different data types from bytes.<br>
<br>
After generating the water level values, we have to use these values as the input of the controller model which has been shown as a block in this model. Finally the results of the model should be sent back via serial communication and we have to convert this value to its corresponding bytes.<br>
<br>
The Following picture shows the structure of the "GISOO-Simulation Environment" for this scenario. The important point that should never be mistaken is the location of the controller model. This model should always be located inside the "GISOO-Simulation Environment". The outside of the "GISOO-Simulation Environment" is our <b>physical environment or Plant</b> and the controller model is not part of the plant!<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/serialcommunication/inside_CoojaPlatform_SerialCommunication.png'>
<p>

<h3>CTP Routing protocol and multiple tanks scenario</h3>
TinyOS codes and Simulink Model can be reached: <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_ctp'>here</a>

This section will explain an advanced version of previous water-tank scenario with some interesting specification. The main important characteristic of this sample has laid on the routing layer but in overall the main specification of this sample can be listed like this:<br>
<ul><li>Using <a href='http://sing.stanford.edu/gnawali/ctp/'>, CTP</a> as a routing protocol in a scenario and simulate it in the GISOO.<br>
</li><li>Creating a scenario with several number of motes and different connections between Simulink an Cooja.<br>
</li><li>Creating a single simulation containing several plants.</li></ul>

In this scenario we simulate a water tank laboratory which contains 10 different double tanks and all of them will work in parallel and be controlled by single central controller. In fact this is a simulation of our water tank laboratory environment at KTH which has been shown in the following picture:<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/CTP/KTH_WaterTanks_Lab.jpg' height='500px'>
<p>

In this scenario since we have 10 sets of double tanks, we will need 10 motes to receive the sensor data from each set, 10 actuators that can actuate the motor in each set separately, 1 mote as a central controller and 17 relays to forward the sensor data to the central controller. In this scenario mote-ids 1 - 10 are sensors, 21 - 30 are actuators, controller is with mote-id 41 and relays are 42 - 58<br>
<br>
On the other hand in order to use CTP as dynamic routing protocol between sensors and controller we modified the code of the simple tank scenario to add the CTP to its structure. You can download the TinOS codes for this scenario from <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_ctp'>here</a>.<br>
<br>
Finally the created scenario in Cooja looks like the following picture. The GISOO plug-in ("GISOO-Simulink plugin") become activated for all the sensors and actuators. The picture also shows the "Radio messages" logger which can be activated from "Tools" menu. This is one of the great plug-ins in Cooja which shows all the radio messages during the simulation with complete details.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/CTP/Cooja_LargeScenario.png'>
<p>

The GISOO environment in Simulink contains of One "GISOO-Simulation Environment" and 10 separate plant that each of them act as one set of double tank. The following picture shows the GISOO environment in Simulink.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/CTP/Simulink_LargeScenario.png'>
<p>

Inside the "GISOO-Simulation Environment" block in Simulink, we added one mote in mirror of each sensor and actuator in the Cooja. It is necessary to emphasize that a mirror mote is a mote with same mote-id  in the other environment.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/samples/CTP/CoojaPaltform_LargeScenario.png'>
<p>

Results for this scenario and some other similar scenarios which contains the interference or mote removal has been explained in our paper: "<a href='http://people.kth.se/~araujo/publications/GISOO_IECON13.pdf'>GISOO: a virtual testbed for wireless cyber-physical systems</a>".