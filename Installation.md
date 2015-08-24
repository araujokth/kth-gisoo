

---

# GISOO's Overview #
Since GISOO is an integration of [COOJA](https://github.com/contiki-os/contiki/wiki/An-Introduction-to-Cooja) and Simulink, you need to install COOJA and Simulink properly first, and then it is necessary to add the **GISOO COOJA plug-in** and the **GISOO Simulink Library** to Simulink. More explanations about these steps can be found in the following steps.

GISOO works in both Linux and Windows machines and so, we provide installation procedures for both OSs.

# COOJA #

COOJA is a mote emulator which has been developed for [Contiki OS](http://www.contiki-os.org/) but also supports [TinyOS](http://www.tinyos.net/) applications. It can simulate motes using actual compiled and executable code Contiki OS or TinyOS code.
The following section explains the COOJA installation procedure.

## COOJA installation for Linux ##

In order to be able to run COOJA on your Linux machine, firstly you need to install the latest Java virtual machine (JVM). The JVM installation procedure can be found [here](http://www.java.com/en/download/help/download_options.xml#linux). You also need to install [Apache Ant](http://ant.apache.org/). You can install it by issuing the following command:

```
sudo apt-get install ant
```

Finally, you can download Contiki from [GIT](https://github.com/contiki-os/contiki), to a local folder in your system, issuing the following commands:
```
cd ~
mkdir -p local/src
cd local/src
git clone https://github.com/contiki-os/contiki
```

Now you have Contiki and COOJA in your computer.

## COOJA installation for Windows ##

The installation procedure for COOJA on Windows can be divided into five steps:
  1. Installing Java Runtime Environment (JRE) and JDK
  1. Installing Ant
  1. Installing Cygwin and path environment configuration
  1. Installing the MSPGCC (The GCC tool chain for MSP430)
  1. Downloading the latest Contiki and run the Cooja in Cygwin

These steps are now explained in detail.

### Installing Java Runtime Environment (JRE) and JDK ###

As COOJA is a Java based simulator, Java Runtime Environment (JRE) is necessary. JDK (Java Development Kit) is needed for the next step (Installing [Ant](http://ant.apache.org/)). The JRE and the JDK can be both downloaded from [here](http://www.oracle.com/technetwork/java/javase/downloads/index.html).

### Installing Ant ###
The build tool Ant, which is a java library and command-line tool, is needed for running COOJA. Ant can be used for this aim and can be downloaded from this [link](http://code.google.com/p/winant/).

### Installing Cygwin and path environmental configuration ###
This tool is needed for running COOJA on Windows and can be downloaded from [here](http://www.cygwin.com/).
By using the setup.exe, Cygwin installation will be started. The recommended path for Cygwin is C:\cygwin. In the step of selecting packages, it is required to change the installation status of the package named “Devel” to “install”, since it will not be installed by default. The rest of the packages you can leave them as Default. Of course, if you have some spare time, you can always install all Cygwin packages.

After installation, the Cygwin binaries path should be added to the PATH environmental variable which can be done by running Cygwin and issuing the following command:

```
export PATH=$PATH:/bin
```

### Installing the MSPGCC (The GCC tool chain for MSP430) ###

In order to run COOJA, [MSPGCC](http://sourceforge.net/apps/mediawiki/mspgcc/index.php?title=MSPGCC_Wiki) (Tool chain for [MSP430](http://en.wikipedia.org/wiki/TI_MSP430)) is required. You can find the download link of MSPGCC for Windows [here](http://sourceforge.net/apps/mediawiki/mspgcc/index.php?title=Install:windows). After extracting it, you need to add the "/bin" and "/msp430/bin" to the environmental variable in Cygwin by issuing these commands:

```
export PATH=$PATH:/cygdrive/c/mspgcc/bin
export PATH=$PATH:/cygdrive/c/mspgcc/msp430/bin
```

### Downloading Contiki 2.6 and running COOJA in Cygwin ###
As the last step, please download **Contiki 2.6** from [here](http://sourceforge.net/projects/contiki/files/Contiki/). After extracting the downloaded file to desired folder, one is now able to run COOJA. This can be done by running Cygwin, and then navigate to

```
cd contiki-2.6/tools/cooja
```
and issuing the command
```
ant run
```

NOTE: It can happen that you run the ant command, that the java folder is not correctly being set. If this happens to you, you can solve it by typing in Cygwin:

```
export JAVA_HOME="C:/Program Files/Java/jdk1.7.0_45"
```

## Testing your COOJA installation ##

In order to test COOJA, we recommend you to go to the Contiki page [here](http://www.contiki-os.org/start.html), and follow the instructions in Step 3: **Run Contiki in simulation**. You should be able to get the same results as shown there if you were successful at installing Contiki and COOJA.

<a href='Hidden comment: 
After these steps are performed, one can Verify the correct installation of COOJA by running an Hello World simulation by performing following steps:
* Open "File > New simulation", and click **Create**
* Open "Mote Types > Create mote type > **Sky Mote Type**"
* Enter a suitable description, e.g., "hello_world_mote"
* Click Browse, and select the Contiki "Hello World" application **: hello-world.c**
* Compile the Contiki library by clicking **Compile**
* When the compilation finishes, load the library and create the mote type by clicking "Create".
* Enter, e.g., **10** and click **Create and Add**
* Press **Start** (or CTRL+S) in the **Control Panel** plugin to start simulating. You now should be able to see the motes communication

'></a>

## Adding the GISOO plugin in Cooja ##

After verifying the COOJA installation, you are now ready to add GISOO plugin to COOJA. First, you have to add the "GISOO-Simulink plug-in" which can be downloaded from [here](https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fgisoo_installation_files%2FGISOO_Simulink_Plugin) to the Cooja. For this aim you have to do three steps:
  * After Downloading the "GISOO-Simulink plug-in" folder, you have to place it on the following path:

```
YOUR_CONTIKI_FOLDER/tools/cooja/apps
```
  * In the second step you have to download our modified MSPSim.jar from [here](https://code.google.com/p/kth-gisoo/source/browse/trunk/files/gisoo_installation_files/mspsim-ModifiedCC2420.jar) and [kth-gisoo/source/browse/trunk/files/gisoo\_installation\_files/mspsim-ModifiedCC2420.jar here] then you should rename the file to the mspsim.jar and substitute this file with the one which exists in your contiki folder in this path:

```
YOUR_CONTIKI_FOLDER/tools/mspsim
```

(This mspsim.jar has been modified to support the DAC communication, also some modificatoin in the cc2420 driver has been done to support 15.4 protocol but if you are not using this protocol you can download this [mspsim.jar](https://code.google.com/p/kth-gisoo/source/browse/trunk/files/gisoo_installation_files/mspsim.jar) which has only modified to support the DAC but with original cc2420 driver and has been tested with Contiki 2.6).
  * In the third step you have to modify the build file in the Cooja folder and add

```
<ant antfile="build.xml" dir="apps/GISOO_Simulink_Plugin" target="jar" inheritAll="false"/>  
```

to the following part (in the following code we already added the mentioned line and you can see it in the last line):
```
<target name="jar" depends="jar_cooja">
    <ant antfile="build.xml" dir="apps/mrm" target="jar" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/mspsim" target="jar" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/avrora" target="jar" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/serial_socket" target="jar" inheritAll="false"/>  
    <ant antfile="build.xml" dir="apps/collect-view" target="jar" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/powertracker" target="jar" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/GISOO_Simulink_Plugin" target="jar" inheritAll="false"/>
  </target>
```


And also add this line:

```
<ant antfile="build.xml" dir="apps/GISOO_Simulink_Plugin" target="clean" inheritAll="false"/>  
```

to the following part (it has already been added in the following lines):

```
<target name="clean" depends="init">
    <delete dir="${build}"/>
    <delete dir="${dist}"/>
    <ant antfile="build.xml" dir="apps/mrm" target="clean" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/mspsim" target="clean" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/avrora" target="clean" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/serial_socket" target="clean" inheritAll="false"/>  
    <ant antfile="build.xml" dir="apps/collect-view" target="clean" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/powertracker" target="clean" inheritAll="false"/>
    <ant antfile="build.xml" dir="apps/GISOO_Simulink_Plugin" target="clean" inheritAll="false"/> 
  </target>
```

# Simulink #

[Simulink](http://www.mathworks.se/products/simulink/) is the most widely used tool for simulation and
model-based control design by Mathworks. Simulink is part of MATLAB and you will have to have a valid MATLAB license to be able to use it.

Note that to be able to run GISOO, you **must** have Simulink installed on your machine.(The Matlb version should not be older than 2013)

## Adding GISOO Simulink Library to Simulink ##

You need to download the GISOO Simulink Library folder from [here](https://code.google.com/p/kth-gisoo/source/browse/#svn%2Ftrunk%2Ffiles%2Fgisoo_installation_files%2FGISOOSimulinkLib%253Fstate%253Dclosed) and add it to you MATLAB path variable. This is done by opening "File/Set path" in MATLAB.