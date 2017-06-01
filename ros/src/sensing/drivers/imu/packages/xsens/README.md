# Prerequisites

* Install the MTi USB Serial Driver
  ```sh
  $ git clone https://github.com/xsens/xsens_mt.git
  $ cd ~/xsens_mt
  $ make
  $ sudo modprobe usbserial
  $ sudo insmod ./xsens_mt.ko
  ```

* Install gps_common
  ```sh
  $ sudo apt-get install ros-distro-gps-common
  ```

# Running the Xsens MTi ROS Node
1. Copy the contents of the src folder into your catkin workspace 'src' folder.
   Make sure the permissions are set to _o+rw_ on your files and directories.
   For details on creating a catkin workspace environment refer to [Creating a catkin ws](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment#Create_a_ROS_Workspace)

2. in your catkin_ws ($CATKIN) folder, execute
   ```sh
   $ catkin_make
   ```

3. Source the environment for each terminal you work in. If necessary, add the
   line to your .bashrc
   ```sh
   . $CATKIN/devel/setup.bash
   ```

4. Initiate the ros core
   ```sh
   $ roscore
   ```

5. Open a new terminal, type
   ```sh
   $ . $CATKIN/devel/setup.bash
   $ rosrun xsens_driver mtdevice.py -m $sm -f $fs # publish sensor data
   ```
   where $fs can be 1,5,10,20,40,50,80,100,200 or 400Hz. This configures the MTi
   to output inertial data and magnetometer data at the set ODR. The maximum
   supported inertial update rate is 400Hz and for the magnetometer it is 100Hz.
   The $sm can be set to 1,2 or 3. This can be used to set the sensor to output
   sensor data or filter outputs.

6. To run the node
   ```sh
   $ rosrun xsens_driver mtnode.py _device:=/dev/ttyUSB0 _baudrate:=115200
   ```
   or
   ```sh
   $ rosrun xsens_driver mtnode.py 
   ```

7. Open a new terminal (do not forget step 3)
   ```sh
   $ . $CATKIN/devel/setup.bash
   $ rostopic echo /mti/sensor/sample
   ```
   or
   ```sh
   $ . $CATKIN/devel/setup.bash
   $ rostopic echo /mti/sensor/imu
   ```

# Troubleshooting

* The Mti1 (Motion Tracker Development Board) is not recognized.
  Support for the Development Board is present in recent kernels. (Since June 12, 2015).If your kernel does not support the Board, you can add this manually

    $ sudo /sbin/modprobe ftdi_sio
    $ echo 2639 0300 | sudo tee /sys/bus/usb-serial/drivers/ftdi_sio/new_id


* The device is recognized, but I cannot ever access the device
  Make sure you are in the correct group (often dialout or uucp) in order to access the device. You can test this with

        $ ls -l /dev/ttyUSB0
        crw-rw---- 1 root dialout 188, 0 May  6 16:21 /dev/ttyUSB0
        $ groups
        dialout audio video usb users plugdev

    If you aren't in the correct group, you can fix this in two ways.

    1. Add yourself to the correct group
        You can add yourself to it by using your distributions user management
        tool, or call

            $ sudo usermod -G dialout -a $USER

        Be sure to replace dialout with the actual group name if it is
        different. After adding yourself to the group, either relogin to your
        user, or call

            $ newgrp dialout

        to add the current terminal session to the group.

    2. Use udev rules
        Alternatively, put the following rule into /etc/udev/rules.d/99-custom.rules

            SUBSYSTEM=="tty", ATTRS{idVendor}=="2639", ACTION=="add", GROUP="$GROUP", MODE="0660"

        Change $GROUP into your desired group (e.g. adm, plugdev, or usb).


* The device is inaccessible for a while after plugging it in
    When having problems with the device being busy the first 20 seconds after
    plugin, purge the modemmanager application.
