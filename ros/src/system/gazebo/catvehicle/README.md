# About
The catvehicle ROS package houses files that utilize the Gazebo simulator, and additional interfaces to the physical CAT Vehicle Testbed.

# Dependencies
* ROS
* obstaclestopper

# catkin workspace and build
In order to use the catvehicle ROS package, you should work within a catkin workspace. If you do not already have one:
```
cd ~
mkdir -p catvehicle_ws/src
cd catvehicle_ws/src
catkin_init_workspace
cd ..
catkin_make
```

At this point, you can extract this package into your src directory
```
cd catvehicle_ws/src
tar xzf catvehicle-x.y.z.tgz
cd ..
catkin_make
```

# Simple tutorial and examples
Follow the tutorials on the CAT Vehicle Testbed group on the CPS Virtual Organization to see how to use the testbed.

# Acknowledgements
## License
Copyright (c) 2013-2017 Arizona Board of Regents
All rights reserved

Permission is hereby granted, without written agreement and without 
license or royalty fees, to use, copy, modify, and distribute this
software and its documentation for any purpose, provided that the 
above copyright notice and the following two paragraphs appear in 
all copies of this software.
 
IN NO EVENT SHALL THE ARIZONA BOARD OF REGENTS BE LIABLE TO ANY PARTY 
FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR CONSEQUENTIAL DAMAGES 
ARISING OUT OF THE USE OF THIS SOFTWARE AND ITS DOCUMENTATION, EVEN 
IF THE ARIZONA BOARD OF REGENTS HAS BEEN ADVISED OF THE POSSIBILITY OF 
SUCH DAMAGE.

THE ARIZONA BOARD OF REGENTS SPECIFICALLY DISCLAIMS ANY WARRANTIES, 
INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
AND FITNESS FOR A PARTICULAR PURPOSE. THE SOFTWARE PROVIDED HEREUNDER
IS ON AN "AS IS" BASIS, AND THE ARIZONA BOARD OF REGENTS HAS NO OBLIGATION
TO PROVIDE MAINTENANCE, SUPPORT, UPDATES, ENHANCEMENTS, OR MODIFICATIONS.

## Authors and contributors
* Jonathan Sprinkle (sprinkjm@email.arizona.edu)
* Rahul Bhadani (rahulkumarbhadani@email.arizona.edu)
* Sam Taylor
* Kennon McKeever (kennondmckeever@email.arizona.edu)
* Alex Warren
* Swati Munjal (smunjal@email.arizona.edu)
* Ashley Kang (askang@email.arizona.edu)
* Matt Bunting (mosfet@email.arizona.edu)
* Sean Whitsitt

## Support
This work was supported by the National Science Foundation and AFOSR under awards 1521617, 1446435, 1262960 and 1253334. Any opinions, findings, and conclusions or recommendations expressed in this material are those of the author(s) and do not necessarily reflect the views of the National Science Foundation.

