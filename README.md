# ROS-Driver

## Introduction
**From Roboteq**

```
 Note*  
 -This ROS driver only supports firmware version 2.0 or 2.0+. 
 -You can check your firmware version from Roborun+ console tab by querying - "?fid". 
 -If firmware is not the latest one then please update it with the latest one available on Roboteq website 
  or contact "techsupport.roboteq@mail.nidec.com".
```
This repository contains the ROS driver for Roboteq controllers. The package requires ROS system to be installed properly to your system  and proper connection of Roboteq controller. For detailed controller setup instructions, please refer to our documentation [here (link died)](?).

**Roboteq CLAIMED!!!**

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

**FACTS:**
- It originally worked at a fixed rate of 5 Hz while querying system's states. I did **major** mofifications in this work, making it work at a frequency you want to.
- They did also specified 3 separate query frequencies in [the config file](config/query.yaml): `frequencyH`, `frequencyL`, and `frequencyG`. However, it's not the case (or quite complicated). Here in this work, I cleaned all of it and only keep a default `frequency` for all queries. It's sufficient for me, and hopefully for you too.
- I only used the `driver.launch` for now. So `diff_odom` is kept as original. Later, I might make it works, but probably by modifying the `roboteq_controller_node`, not putting in a separated file to make it a little bit efficent.


**Note**: This package is tested on XDC2460. In general, it uses serial communication so feel free to test it on your roboteq device. Please let me know it you do so, whether it works, any issues. Thank you.



## Installation
```bash
cd YOUR_WS/src/
git clone https://github.com/DoanNguyenTrong/roboteq_controller_ros.git
cd YOUR_WS
catkin build roboteq_controller
source devel/setup.bash
roslaunch roboteq_controller driver.launch
```
