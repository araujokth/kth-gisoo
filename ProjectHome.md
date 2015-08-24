## What is GISOO? ##

The increasing demand for wireless cyber-physical systems (CPS) requires correct design, implementation and validation of computation, communication and control methods. Traditional simulation tools, which focus on either computation, communication or control, are insufficient when the three aspects interact. Efforts to extend the traditional tools to cover multiple domains, e.g., from simulating only control aspects to simulating both control and communication, often rely on simplistic models of a small subset of possible communication solutions.

We developed GISOO, a virtual testbed for simulation of wireless cyber-physical systems that integrates two state-of-the art simulators, [Simulink](http://www.mathworks.se/products/simulink/) and [COOJA](http://www.contiki-os.org/start.html). GISOO enables users to evaluate actual embedded code for the wireless nodes in realistic cyber-physical experiments, observing the effects of both the control and communication components. In this way, a wide range of communication solutions can be evaluated without developing abstract models of their control-relevant aspects, and changes made to the networking code in simulations is guaranteed to be translated into production code without errors.

## What can I find here? ##

In this google-code project you can find GISOO's code, examples and all related documentation. We will be adding many example codes so that you can develop your own wireless CPS experiments!

GISOO's examples are currently developed for TinyOS and telosb, tmote sky or CM5000 nodes, which all have an [MSP430 uC](http://www.ti.com/lsds/ti/microcontroller/16-bit_msp430/overview.page?DCMP=MCU_other&HQS=msp430) and a [CC2420 radio](http://www.ti.com/product/cc2420). However, through COOJA, GISOO can support many other platforms. GISOO also supports the most popular wireless sensor networks operating systems: [TinyOS](http://www.tinyos.net/) and [Contiki OS](http://www.contiki-os.org/start.html). However, we are still working on developing our examples to Contiki OS.

## Help and suggestions? ##

You are welcome to contact **Behdad Aminian** (behdad@kth.se) and **José Araújo** (araujo@kth.se) if you have any doubts, need help with GISOO or you have any suggestions to improve it!

## GISOO Publication ##

If you write a paper and want to make a reference to GISOO, you can cite the following publication:

B. Aminian, J. Araujo, M. Johansson and K. H. Johansson, "**GISOO: A virtual testbed for wireless cyber-physical systems**", IECON 2013 - 39th Annual Conference of the IEEE, pp. 5588-5593, November 2013.

We presented GISOO at [IEEE IECON 2013](http://www.iecon2013.org/) on a special session on  Industrial Wireless Communication and its Applications, organized by M. Gidlund and J. Åkerberg . The paper is available [here!](http://people.kth.se/~araujo/publications/GISOO_IECON13.pdf)

## The Team ##

**[José Araújo](http://people.kth.se/~araujo/) - Postdoctoral researcher - araujo@kth.se**

**[Mikael Johansson](http://people.kth.se/~mikaelj/) - Professor - mikaelj@kth.se**

**[Karl Henrik Johansson](http://people.kth.se/~kallej/) - Professor - kallej@kth.se**

## Alumni ##

**[Behdad Aminian](http://people.kth.se/~behdad/) - Research engineer - behdad@kth.se - currently at Ericsson AB, Sweden**



Department of Automatic Control - KTH The Royal Institute of Technology, Stockholm, Sweden