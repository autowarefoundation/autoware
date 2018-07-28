***
*** LDMRS Example
***
Date: Sept. 23, 2013
Last update: December 18, 2014

An example project for the LD MRS laserscanner. Intended to be used under Linux.


To build the project
====================
A) Manual build
To manually create the executable, follow these steps:
- Open a command shell
- Change to the build-release folder (e.g. "cd /home/myname/LDMRS_Example/build-release")
- Call make ("make").
Now, there should be an executable called LdmrsExample.release.

- To clean up the object files, call "make clean".

b) Automatic build
The folder contains CMake files. Applications such as CMake or KDevelop can use them directly to
create a project.
Example:
- Make sure cmake and ccmake are available
- Change to the build-release folder (e.g. "cd /home/myname/LDMRS_Example/build-release")
- Remove the manual makefile (file "makefile")
- Call ccmake: "ccmake .."
- Enter "Release" as the build type
- Type "c" to configure and "g" to generate, then "q" to quit
- Call "make" to build the program. The executable (defalt name is LDMRS_Example) is then located in the subdirectory "src".


Source code
===========
Note that the source code is provided as-is without warranties of any kind. It is intended for
demonstration purposes, not for efficiency.
All source code is located in the src folder:
LDMRS_Examples
 - build-release
 - src
   - application (the application(s) working with the data)
   - datatypes (all kind of datatypes used in the software)
   - devices (the LDMRS device class and its helpers)
   - interfaces (the TCP interface class)
   - sopas (some SOPAS support stuff)
   - tools (some useful tools)
   - main.cpp (THE main function).

General description
===================
The main function creates a manager object that receives all data from the device(s) and
forwards it to the attached applications. Then, it creates the application(s) itself, and
finally the devices. Note that in this implementation, the devices (in this case, the LD-MRS)
are not supplied with the necessary parameters, but they are hard-coded in the device. Thus,
only one device can be created.

LDMRS
=====
The LD-MRS laserscanner is interfaced by two different protocols. Scans and Object data (as
well as their configuration, such as the scan area) is sent via the generic LUX tcp port 12002,
while the field configuration, eval case configuration and evalCaseResults are read via the
SOPAS interface (port 2111 or 2112). Note that the LD-MRS only supports the CoLa-B protocol.

Application "MrsApp"
====================
The MRS-App connects to an MRS, reads its configuration and receives all incoming data.

Application "SectorChangeApp"
=============================
The SectorChangeApp demonstrates the usage of the FlexRes ("Flexible resolution") feature. It defines
several sectors with different scan resolutions and configures the sensor accordingly.
This application works only with firmware that supports FlexRes.

Application "FieldApp"
======================
The FieldApp demonstrates the usage of the SOPAS interface. It removes all fields and eval cases, then
creates a new field and matching eval case. It also demonstrates loggin in and out, and flashing of the
configuration.

Application "NtpTimeApp"
========================
This application demonstrates the setting of a system time in the sensor. It waits for 2 seconds, then
sets the sensor-internal system time to January 1st, 2000.

Application "ScanpointCoordinateApp"
====================================
The ScanpointCoordinateApp demonstrates the use of scanpoint coordinates and allows to verify the coordinate calculation (done in LuxBase.cpp).
It also demonstrates how to distinguish between 4- and 8-layer-scanners and their mirror sides.

<End of file>
