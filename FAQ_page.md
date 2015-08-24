

---


# Frequently Asked Questions #


---

## What do I need in order to use GISOO? ##

You will need to have:

  * A machine running either Linux or Windows

  * [MATLAB](http://www.mathworks.se/products/matlab/), [Simulink](http://www.mathworks.se/products/simulink/)

  * COOJA installed (comes with Contiki OS)

  * TinyOS installed (optional)

Want to install GISOO? Follow the installation instructions on the "Installing GISOO" link!

## What if the mote emulation in COOJA does not work as in the real motes? ##

This could be because there are some incomplete, incorrect or missing drivers in MSPsim. Please let us know if this happens to you so we can be aware of it. Debugging MSPsim is pretty straightforward if you know a bit of Java and know how to use printfs.

You have to download our modified MSPsim to your machine from [here](https://code.google.com/p/kth-gisoo/source/browse/trunk/files/gisoo_installation_files/mspsim-withDAC.zip) which has been modified to support DAC communication and tested with Contiki2.6)

Then, you can go and modify the drivers you need to modify. For example, the CC2420 driver is located in the folder "se/sics/mspsim/chip"

Do your required modification in the file then open a terminal inside the  main folder of MSPsim and type
```
ant jar
```
this command will compile the new code and create a mspsim.jar file.
Copy the .jar file and place it in the following folder "YOUR\_CONTIKI\_FOLDER/tools/mspsim". Then just run COOJA again to test your new driver!
You can fined our mspsim with the new cc2420 driver in [here](https://code.google.com/p/kth-gisoo/source/browse/trunk/files/gisoo_installation_files/mspsim-ModifiedCC2420.zip) this new driver can support 15.4 radio protocol as well.

If you are interested to see the original code for mspsim you can find it by following this commands but you should consider that the original mspsim is not proper to be used in GISOO since it doesn't support the DAC communication.

```
cd ~
cd local/src
git clone https://github.com/mspsim/mspsim
```

## What if there is a driver in MSPsim that is incomplete, incorrect or missing? ##

Please let us know if this happens to you so we can be aware of it. If this is the case, you have to install MSPsim and either fix MSPsim, or develop your new driver. Please check "What if the mote emulation in COOJA does not work as in the real motes?" for more information on how to install and compile MSPsim.

## How to create a specific mote-id in Cooja? ##
Since Cooja creates motes in order (It means that the generated mote-id will only be one after the other), the only way to create a mote with a specific Id is to create all the previous motes first and then create your mote with that id. For instance if you need only one mote in your simulation with id=4, first you need to crate 4 motes from your mote's type and then you can delete the first three ones.
(Mote-ids in Cooja will be shown as a number on the mote circle.)

## How to use the printf command in Cooja and GISOO? ##
In Cooja there is a "Mote Output" plug-in (which will be appear by default when you run the Cooja). This plug-in will show all the printed data with the printf command. The only point which should be considered is that you use the "SerialPrintfC" component  instead of "PrintfC" in your TinyOS code, otherwise you will receive several unwanted character in your printed message.

## Can we record the times and values of reading ADCs and writing DAC in GISOO? ##
Yes. "GISOO-Simulink plugin" in Cooja by default contains a folder by name of "logs" which contains two text files: "ADCTime.txt" and "DACTime" and will record:
Mote-pin-id,   simulation time(in ms),    ADC/DAC values
in three different columns. If you do not need these logs, you can delete the folder, so these data will not recorded any more.(It may improve your simulation speed). By creating that folder at any time you can again record those values.

## Why 1 second in TinyOS is not equal with 1 second in Cooja? Can it affect the result of Simulation? ##
Cooja by default has been developed for Contiki's applications. In Contiki each second has been considered as 1000 milliseconds. But in TinyOS each second is 1024 milliseconds. This different will cause that any second in TinyOS application appear as 977 ms (1000/1024). But since this ratio is constant for all variety of times it should not make any affect in the result of the simulation.

## What should I do if I had problem for Serial Communication in GISOO? ##
if you want to send data from Cooja to Simulink first be sure that your serial data has 16 bytes structure. (The structure of serial data has been explained in [Serial Communication](https://code.google.com/p/kth-gisoo/wiki/Samples?ts=1385498264&updated=Samples#Serial_Communication))
But for serial communication from Simulink to Cooja the message should contains all the header and footer to be received by TinyOS application. The structure of serial messages has been explained in [TEP 113](http://tinyos.cvs.sourceforge.net/viewvc/tinyos/tinyos-2.x/doc/html/tep113.html). The designed block for creating the Serial message with header and footer has been shown in the [SkyMote](https://code.google.com/p/kth-gisoo/wiki/technical_details#Sky-Mote) explanations. Inside this block there are some constant blocks which provide the value the framing bytes, sender address, receiver address, Group-id and so on. In general simulation you should not need to change these values and the only which you may need to change should be AM packet type that in GISOO you can set it as one of the SkyMote parameter. But in rare situation you may need to change the other constant values. Just check and see if these change of value is needed in your case or not. If you still have problem please let us know, to fix the problem.

## What should I do if I had Cooja on my computer but I faced some problem to add the GISOO's plug-in to my previous Cooja? ##
First check if you use the correct version of Cooja or not. (The modified mspsim.jar has been used by Contiki 2.6) In the next step,Cooja save its configuration in a hidden file by name of .cooja.user.properties. These settings are stored in your home directory such as in /home/myuser or C:\Documents And Settings\myuser.
It is better to to delete this file (you can do it by the "rm" command)and then try to add GISOO plug-in again.

## In Simulink I always receive unsigned values but I need to use signed values (signed int32) what I should do for that? ##
The only change that you have made inside the " Bytes to uint32 Convertor" to convert the result from uint32 to int32 (this change is not required when you are using the (u)int16.
For this change you should just go inside the " Bytes to uint32 Convertor" block and in the property window of "Sum of Elements" change the "accumulator data type" to the int8.

<p align='center'>
<img src='https://kth-gisoo.googlecode.com/svn/trunk/images/FAQ/SignedInt32.jpg'>
<p>