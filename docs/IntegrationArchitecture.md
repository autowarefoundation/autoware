## Autonomous Driving Software Stack

![autoware overview](/home/punnu/Desktop/Autoware/docs/images/autoware_overview.png)

The self-driving car software stack as presented in the picture above is divided into 6 categories: 
Sensing, Computing, Decision, Planning, Actuation(DBW) and Others. 

Each of these categories may contain multiple modules such as localization, detection and prediction in Computing categories. Additionally, each module has multiple software packages.

Category -> Module -> Packages 


## Autoware Autonomous Driving Concept
**(revise: should we use images from autoware ?)**

The modules from the software stack above shows a high-level autonomous driving concept that can be exhibit further in the following figure.

![DrivingConcept](/home/punnu/Desktop/Autoware/docs/architecture/DrivingConcept.png)


For the simplicity, we organize the figure with the following a groups of functionalities, namely .....


Outline the functionalities of each module.





## System Integration
Expanding from the previous figure, the high-level concept of self-driving is presented further of how
each elements put together to enable self-driving capability.



![SystemIntegration](/home/punnu/Desktop/Autoware/docs/architecture/Integration.png)

we show the abstract design of system integration with key elements implemented in the software stacks.



## Overview of Autoware Path Planning Strategy



![PathPlan](/home/punnu/Desktop/Autoware/docs/architecture/pathplan.png)




## Examples: 
We show some integration use cases based on a selection of motion planing methods. 
1. Lattice motion planning

![Lattice](/home/punnu/Desktop/Autoware/docs/architecture/ComponentDiagram_Lattice.png)









2. A* motion planning



