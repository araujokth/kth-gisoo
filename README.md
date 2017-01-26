### What is GISOO?

The increasing demand for wireless cyber-physical systems (CPS) requires correct design, implementation and validation of computation, communication and control methods. Traditional simulation tools, which focus on either computation, communication or control, are insufficient when the three aspects interact. Efforts to extend the traditional tools to cover multiple domains, e.g., from simulating only control aspects to simulating both control and communication, often rely on simplistic models of a small subset of possible communication solutions.

We developed GISOO, a virtual testbed for simulation of wireless cyber-physical systems that integrates two state-of-the art simulators, Simulink and COOJA. GISOO enables users to evaluate actual embedded code for the wireless nodes in realistic cyber-physical experiments, observing the effects of both the control and communication components. In this way, a wide range of communication solutions can be evaluated without developing abstract models of their control-relevant aspects, and changes made to the networking code in simulations is guaranteed to be translated into production code without errors.

### What can I find here?

Here you can find GISOO's code, examples and all related documentation. 

GISOO's examples are currently developed for TinyOS and telosb, tmote sky or CM5000 nodes, which all have an MSP430 uC and a CC2420 radio. However, through COOJA, GISOO can support many other platforms. GISOO also supports the most popular wireless sensor networks operating systems: TinyOS and Contiki OS. 

### Help and suggestions?

The code is currently under no maintanence from the development team. However, you are very welcome to contribute to this project. If you want to be added as a collaborator please contact José Araújo at jose.araujo@ericsson.com

### GISOO Publication

If you write a paper and want to make a reference to GISOO, you can cite the following publication:

B. Aminian, J. Araujo, M. Johansson and K. H. Johansson, "GISOO: A virtual testbed for wireless cyber-physical systems", IECON 2013 - 39th Annual Conference of the IEEE, pp. 5588-5593, November 2013.

We presented GISOO at IEEE IECON 2013 on a special session on Industrial Wireless Communication and its Applications, organized by M. Gidlund and J. Åkerberg . The paper is available [here](http://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=6700049)!

### Development team

[José Araújo](www.josearaujo.org) - PhD student - araujo@kth.se - currently Researcher at Ericsson Research, Stockholm

Behdad Aminian - Research engineer - behdad@kth.se - currently at Ericsson, Stockholm

[Alessandro Luppi](https://github.com/AlexLup) - PhD student at TU Graz

### Supervisors

Mikael Johansson - Professor - mikaelj@kth.se

Karl Henrik Johansson - Professor - kallej@kth.se

Department of Automatic Control - KTH The Royal Institute of Technology, Stockholm, Sweden
