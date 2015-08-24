

---

# Overview #
GISOO is an integration of Simulink and COOJA for performing simulation of wireless cyber-physical systems. Thus, each part of the wireless cyber-physical system can be implemented as follows:
  * the **physical system model** is implemented in Simulink.
  * the **wireless sensors, actuators** and **relays** (from physical layer to application layer) are emulated in COOJA
  * the **estimator/controller** can be implemented in either a wireless node in COOJA, or emulated in a CPU in Simulink. The implementation in Simulink allows for a faster design and evaluation since no recompilation of wireless nodes code is necessary.

One of our main goals is code**you upload to the real wireless nodes for performing a real experiment. For this to be achieved, GISOO provides the following interfaces:**

  * **Analog-to-digital converter (ADC)**, for interfacing wireless nodes with real sensors
  * **Digital-to-analog converter (DAC)**, for interfacing wireless nodes with real actuators
  * **Serial interface**, for allowing communication to sensor, actuators and processing units (CPUs, embedded computers, microcontrollers, etc).

In this section you can find all the information on how to use GISOO and a simple example which can help you build your own applications from.

**Note:** As standard in the wireless sensor network literature, we will the terms _wireless node_, _wireless mote_, _wireless sensor node_ or simply _node_ or _mote_ to mean a low-rate wireless device communicating using a IEEE 802.15.4 radio.

---


# Compiling your wireless node code #

First, you should compile the code which you want to upload to your wireless nodes. You can either use TinyOS code or Contiki code for your applications. However, we remind you that at the moment all our examples are based on TinyOS.

In TinyOS you compile your application in a wireless node by executing for eaxmple: "make telosb" for a telosb/tmote sky/cm5000 device. When you do such operation, a "build" folder is automatically created in the application folder, which contains some files including the "main.exe". This file will be used in COOJA as the mote application. You can find a TinyOS installation manual [here](https://code.google.com/p/kth-wsn/downloads/detail?name=GettingStartedWithTinyOS.pdf&can=2&q=‎) and also check [TinyOS tutorials](http://tinyos.stanford.edu/tinyos-wiki/index.php/TinyOS_Tutorials) For more detail about its commands.

Note that you can load several distinct wireless nodes with different applications in the same COOJA simulation.

# COOJA setup #

In order to use GISOO, we must start COOJA, create a new simulation and include the required wireless nodes. Particularly, we must connect the sensors, actuators and all of the wireless nodes which have a direct interface to Simulink. For example, a sensor is likely to take ADC measurements from the physical system (Simulink model), and an actuator is likely to send a control signal (either DAC or serial communication) to the actuator which affects the physical system. We explain these steps below.

## COOJA Execution ##
_**Note:** We now present the execution details in a Windows system, however, we note that the execution for a Linux system is identical. The only difference comes from the fact that Cygwin is used in the Windows system, while the terminal is used in the Linux system_

For executing COOJA, open a shell window of Cygwin and navigate to COOJA's directory. In this example, Cygwin has been installed at C:\cygwin  and the Contiki folder which contains COOJA, is located at C:\contiki-2.6. This step is shown in the screenshot below. Finally in the COOJA's folder you should use execute “ant run” command to run COOJA.

<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/CoojaExecution.png' height='282px' width='837px'>
<p>

<h2>Create a new simulation</h2>
To start a new simulation, create a new simulation environment by going to the file menu as shown in the picture:<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/NewSimulation.png'> <p>

In the window of new simulation creation, you can select a name for your simulation and select your desired radio medium. You can also select the random seed for your simulation, which should be always the same if you want to perform exactly the same simulation every time. The "Create new simulation" window is shown below:<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulation_Start_config.png'>
<p>
After following these steps, the new simulation environment will be created which will result in the following:<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/CoojaEnv.png'>
<p>

<h2>Adding motes to the simulation</h2>

After creating the new simulation environment you have to add all the wireless nodes to the simulation. Although COOJA supports different types of nodes and operating systems, we used the Sky Mote (MSP430-based node) and TinyOS in all of our experiments. However, one can simulate nodes using btoh Contiki OS and TinyOS at the same time in COOJA.<br>
<br>
In order to add a new mote to the simulation you can use the Motes menu and navigate to "Motes>Add motes>Create new mote type>Sky mote" as shown in the following image:<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/MoteCreation.png'>
<p>

In the opened window you should follow these steps:<br>
<ol><li>Select a name for you mote<br>
</li><li>Use the browse button to find the "main.exe" of the application that you want to load in your mote<br>
</li><li>Create the mote</li></ol>

These steps are shown in the following image:<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/MoteCreationSteps2.png'>
<p>

After selecting the "main.exe" and creating the mote, the next window will ask you about the number of desired motes from that application and the location of the motes which by default will be selected randomly.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/MotePositioning.png'>
<p>

You can repeat this procedure for all nodes and all different applications you would like to simulate.<br>
<br>
Finally, after the previous procedure, the virtual mote will be created.<br>
<br>
<b>Note that the number which has been appeared on each mote is its mote-id and it has been selected automatically (in-order) which is not configurable by the user</b>. This is important to remember since in many applications each nodes operation will depend on their node-id. <b>For these type of scenarios you should create motes in an order that each mote receive your desired id.</b>

<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/CreatedMote2.png'>
<p>

<h2>Activating the GISOO COOJA Plug-in</h2>

In order to connect motes in COOJA to Simulink you need to use the GISOO COOJA plug-in which you have previously downloaded from <a href='https://kth-gisoo.googlecode.com/svn/trunk/files/gisoo/GisooLib.zip'>here</a> and added to your COOJA folder (this has been explained in the installation procedure)<br>
<br>
In order to use this plug-in, when running COOJA for the first time, you need to select and activate the plug-in in the COOJA extension list which you can reach and select in "Setting-> COOJA extensions", as depicted in the next image:<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/CoojaExtensionsSetting.png'>
<p>

<h2>Executing the GISOO COOJA plug-in</h2>

In each wireless node which interfaces with Simulink (reads data from it, or writes data to it), you have to right click on that mote and select "Mote tools Sky -> GISOO COOJA plug-in". <b>This plugin allows the communication between the mote in COOJA and the virtual mote in Simulink.</b> You can see the procedure in the next image:<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/SimulinkConnectorSelection.png'>
<p>

As seen in the previous image, GISOO COOJA plug-in is created for the selected mote. The same node-id is created for both the mote in COOJA and the virtual mote in Simulink.<br>
<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/SimulinkConnectorPlug-in.png'>
<p>

<h1>Simulink setup</h1>

To follow these steps it is assumed that the GISOO Simulink Library was added already to MATLAB. Now you have access to all the component which you need to be able to run your simulation with GISOO.<br>
<br>
Start by creating a new model from MATLAB by going to "File>New>Model", in order to open Simulink. Now in your model window, go to "View>Library Browser" and you should be able to see the GISOO Library and shown in the following image:<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/GISOOLib.png'>
<p>

<h2>Adding the GISOO Simulink Environment (GSE)</h2>

The first step in order to perform the simulation is to add the "GISOO Simulink Environment" (GSE) block to your simulation environment. This block is the place that allows for GISOO simulations to take place and where the interface between Simulink and COOJA takes place. It will contain all the necessary virtual motes and also handle the synchronization and communication between Simulink and COOJA. The next image shows the GSE block.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/CoojaPlatform.png'>
<p>


<h2>Adding a mote to the GSE</h2>

In the next step, the necessary motes are be added inside the GSE block. This can be performed by double clicking on the GSE block and drag a "SkyMote" block from the "GISOO Library" to the GSE block.<br>
<br>
Note that on the first attempt to add a mote inside the GSE you may face a message which ask you to disable the link to the library and it should be done by clicking on the "disable" part in the pop-up message, as it has shown in the following picture. If you did not see this message you can just add your mote inside the GSE without any concern.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/LinkDisable.png'>
<p>

Finally, you should add all the motes which were enabled with GISOO COOJA Plug-in in COOJA to the GSE. Thus, all nodes that need to be in contact with the plant such as wireless sensors or wireless actuators or other motes which interface with computing nodes in Simulink (e.g., through serial communications), such as a controller implemented in Simulink, should be added to the GSE.<br>
<br>
After adding all the virtual nodes to this environment, it is necessary to set their node-id by double clicking on them. This number should be set to the the same id of their mirror motes in COOJA. As expected, GISOO only allows for a mote in Simulink to communicate with the mote in COOJA if both have the same node-id.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/AddingMote.png'>
<p>

<h2>Adding ADC/DAC connectors</h2>

For each node, according to its action you should add the required ADC or DAC connector component. For instance if you have added a mote to operate as a sensor which should measure data on the ADC0 pin (input to the node), then you have to connect the ADC-Connector component to ADC0 input of that mote, or if you want to receive some actuation data on the DAC0 (output of the node), you need to add the DAC connector component on the DAC0 output of your mote. These components and their location is shown in the following picture.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/ADC-DAC-Connectors.png'>
<p>



<h2>Adding the plant/physical system model</h2>

In the final step, we need to add a physical model of our plant which can be modeled in a Discrete State-Space block outside the GSE. We now have different number of input and output from our GSE block, which has been created by the ADC-connector or DAC-connector components from the previous step.<br>
<br>
Now in this step the outputs from the GSE should be used as the input for our plant model and the output of the plant model should be used as the input for the "GISOO-Simulation Environment". The following picture shows a simple environment and the interaction between the GSE and the plant model.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/AddingPlant.png'>
<p>


<h1>Configuration and running the simulation</h1>

After preparing the simulation environment for the very last configuration, you have to setup the Simulink simulation to act in a fix-step instead of variable-step. You can apply this configuration in the "Configuration parameter" window which is accessible from "Simulation->Model Configuration Parameter" and finally change the type in the solver option to the fix-step. Now you can select the size of the sample time which we usually configure to "0.001" s to have an accuracy of one millisecond (1 ms).<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/Simulink/Configuration.png'>
<p>

Now that both sides of the simulation environment(COOJA and Simulink) have been configured we are ready to run the simulation. We have to <b>first start the simulation in Simulink and then start the COOJA simulation</b>. In this step first Simulink will start its simulation for very first milliseconds and then it will <b>wait</b> until it receives any incoming data from COOJA. The only important point in this action is that if Simulink cannot receive the message from COOJA in less than few seconds it will consider it as a problem and will go out from the simulation execution loop, so it is necessary to run the COOJA in few second after Simulink.<br>
<br>
<h1>Simple Scenario</h1>

We now present an example of a closed-loop control experiment in GISOO.<br>
<br>
The wireless double-tank system is shown in the next figure. Water<br>
is pumped from a reservoir to the upper tank, from which it flows through an outlet to the lower tank. Water from the lower tank flows through a similar outlet back to the reservoir. The pump is controlled<br>
by an actuation signal, which is a control voltage applied to the pump motor. The water levels in both tanks are measured using pressure sensors.<br>
<br>
For sensing and actuation we use sky wireless nodes. These motes contain ADCs (analog-to-digital converters) and DACs (digital-to-analog converters), which are used to connect to the sensors and actuators in the physical system.<br>
<br>
The aim of this scenario is to create such a system which can stabilize the water level in the lower tank on constant level (in this example, the level is set to 5cm). The system is composed by 4 different devices:<br>
<br>
<ul><li>Sensor Node: A sky node is connected to the pressure sensors from the tank process, and takes measurements using the ADC and transmits this information to the Relay node.</li></ul>

<ul><li>Relay Node: Another node is programmed to relay messages between the Sensor Node and the Controller Node. This introduces multi-hop communications.</li></ul>

<ul><li>Controller Node: The control algorithm is implemented inside the wireless node. This node receives measurements from the Relay Node, computes the actuation value and transmits an appropriate control action to the Actuator Node.</li></ul>

<ul><li>Actuator Node: Another node is connected to the pump motor using the DAC. This node receives the control signal from the controller and applies this voltage to the pump in the tank setup.</li></ul>

For this sample, you can download the prepared codes for these motes from <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_simple_scenario'>here</a>.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/helloworld/WaterTanksSystem.png'>
<p>

In order to create such a system in GISOO we have to:<br>
<ul><li>Create all the nodes in COOJA<br>
</li><li>Enable the GISOO COOJA plug-in in the nodes which interface the plant<br>
</li><li>Create a proper GSE in Simulink including all the nodes that interface the plant.<br>
</li><li>Finally, we have to make a model of the plant in Simulink and connect it to the GSE.</li></ul>

In the first step we have to create a simulation with four nodes, as it is shown in the next figure. In this scenario, each node should have its special id as follows:<br>
<ul><li>Sensor: 1<br>
</li><li>Relay: 2<br>
</li><li>Controller: 3<br>
</li><li>Actuator: 4<br>
<b>To create motes with these ids, the order of mote creation in COOJA, should be the same as the motes' id.</b></li></ul>

After adding these nodes, since the sensor should sense the water levels from the plant and the actuator should be able to actuate the plant, both of them should be connected to Simulink. It is then needed to activate the "GISOO COOJA Plug-in" for both of these nodes.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/helloworld/SimpleTanks_Cooja.png'>
<p>

In the next step, you have to create a Simulink model, add a GSE block and add two sky node blocks from the GISOO library. Tese node blocks are the mirror nodes for the Sensor and Actuator in COOJA. Thus, they must have the same node id as the nodes in COOJA. This step is shown in the next figure.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/helloworld/SimpleTanks_CoojaPlatform2.png'>
<p>

In order to create a model of the plant we can use the "Discrete State-Space(Plant)". For this aim we can write a MATLAB script and add it to the "initFcn" of callbacks in the Modelproperties.<br>
<br>
The required MATLAB script (Simulink model) for this experiment can be downloaded from <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_simple_scenario%2FSimulinkModel%253Fstate%253Dclosed'>here</a>.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/helloworld/SimpleTanks_Simulink.png'>
<p>

Finally, the simulation environment is ready and you can run the simulation by running Simulink first, followed by starting COOJA.<br>
<br>
Although you can save all data from Simulink or COOJA for further analyzes, you may also monitor online the signals in both simulators. The next two figures show the changes in the water levels and actuation values during the experiments happening online.<br>
<br>
<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/helloworld/WaterLevels.png'>
<p>

<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/how_to_use_GISOO/helloworld/ActuationVslues.png'>
<p>

You can download all of the required codes (node codes, plant script, and Simulink model) from <a href='https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fexamples%2Fwater_tanks_simple_scenario'>here</a>. Then you just need to create the COOJA simulation and run the simulation!