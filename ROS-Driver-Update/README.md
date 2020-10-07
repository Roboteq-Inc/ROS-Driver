# ROS-Driver
# Quick Start Guide
```
 Note*  
 -This ROS driver only supports firmware version 2.0 or 2.0+. 
 -You can check your firmware version from Roborun+ console tab by querying - "?fid". 
 -If firmware is not the latest one then please update it with the latest one available on Roboteq website 
  or contact "techsupport.roboteq@mail.nidec.com".
```
This repository contains the ROS driver for Roboteq controllers. The package requires ROS system to be installed properly to your system  and proper connection of Roboteq controller. For detailed controller setup instructions, please refer to our documentation [here](https://www.roboteq.com/index.php/docman/motor-controllers-documents-and-files/documentation/user-manual/272-roboteq-controllers-user-manual-v17/file).

First, clone this repository to catkin_ws/src 
```
git clone https://github.com/Roboteq-Inc/ROS-Driver.git
```

The `Roboteq motor controller driver` is the package for the Roboteq ROS-driver. Make sure not to change package name as it will change the definition in the Cmake and Package files. Open new terminal and copy these steps -

```
cd catkin_ws/
source devel/setup.bash
roslaunch roboteq_motor_controller_driver driver.launch
```

The roboteq driver is designed to be dynamic and users can publish the controller queries as per their requirements. The publishing queries is not limited to any value. By default total 9 queries are published by launching this driver. Users can change or add queries in configuration file. For that go to config/query.yaml

```
frequencyH : 50   #higher frequency (value is in ms)
frequencyL : 100  #lower frequency
frequencyG : 100  #frequency for general queries

queryH:
 motor_amps : ?A 
 motor_command : ?M
 encoder_count : ?C
 encoder_speed : ?S  #these queries will publish with higher frequency and users can add other queries below encoder_speed.

queryL: 
 error : ?E 
 feedback : ?F 
 battery_amps : ?BA
 power : ?P          #these queries will publish with lower frequency and users can add other queries below power.
 

queryG: 
 fault_flag : ?FF  
# status_flag : ?FS
# firmware_id : ?FID  Users can add queries which do not require channel number under queryG tab. 
```
