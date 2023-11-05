# microros_imu_publisher
Original - A simple Micro-ROS publisher implementation with ESP-32 dev board &amp; Arduino IDE

## Micro-ROS publisher with Nano RP2040 Connect 

### Micro ROS on Host (Linux) 
#### Get micro-ros-setup package
https://github.com/micro-ROS/micro_ros_setup.git
##### Summary:
  `source /opt/ros/$ROS_DISTRO/setup.bash`  
  `mkdir uros_ws && cd uros_ws`  
  `git clone -b iron https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup`  
  `rosdep update && rosdep install --from-paths src --ignore-src -y`    
  `colcon build`    
  `source install/local_setup.bash`   
#### Create agent
  ##### Download micro-ROS-Agent packages
    ros2 run micro_ros_setup create_agent_ws.sh
  ##### Build step - build packages for agent
    ros2 run micro_ros_setup build_agent.sh
    source install/local_setup.bash
#### Run micro-ROS app
  ##### Run a micro-ROS agent
    ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
  
  ##### In new terminal, run micro-ROS node
    source /opt/ros/$ROS_DISTRO/setup.bash
    source install/local_setup.bash
  ##### Use RMW Micro XRCE-DDS implementation
    export RMW_IMPLEMENTATION=rmw_microxrcedds
  ##### Run a micro-ROS node
    ros2 run micro_ros_demos_rclc ping_pong


### Micro ROS on Arduino 
https://adityakamath.github.io/2022-06-19-microros-examples/  

Download the latest release for ROS2 Humble as a .zip file (https://github.com/micro-ROS/micro_ros_arduino/releases)   
Add it to the Arduino IDE using the ‘Include library’ option.   
Then, code can be compiled and uploaded in the traditional way on the Arduino IDE.  

### Nano RP2040 on Arduino IDE
https://docs.arduino.cc/tutorials/nano-rp2040-connect/rp2040-imu-basics  

NOTE: Arduino IDE on Linux is unable to upload codes to Nano RP2040 Connect. THe code compiles but when uploading the RP2040 board is disconnected.  
Proposed solution is either using MacOS to upload the code or copying the code file to the RP2040 manually ( Need more data )  

- Install drivers for Nano RP2040 Connect: Tools -> Board -> Board Manager -> Search "Arduino Mbed OS Nano Boards" -> Install  
- Install required library: Tools -> Manage Libraries -> Search "Arduino_LSM6DSOX" -> Install  
- Write code to access accelerometer and gyroscope data  

## Accessing published data on Host 
The Arduino Nano RP2040 Connect is connected to host computer with a USB so serial connection exists

### Run Micro-ROS Agent 
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
![image](https://user-images.githubusercontent.com/94715242/219905559-a5b8892d-9ea5-4479-926b-c9e814b2274f.png)

 Note: If you get Open Device Error ( ErrNo 13 ), add your user to dialout group ( dialout has permissions to access the serial ports)

 sudo usermod -a -G dialout your_username

 To make sure the user is completely added, check the groups in terminal. 
 `groups`  
 This should give a list with 'dialout'. You might need to restart the system a few times until this group appears.
 
 
### Disconnect and connect the board
![image](https://user-images.githubusercontent.com/94715242/219905603-d9f27bff-201d-434a-8687-d9df25faeccc.png)
  
  The client, session, topic,and publisher will be created.  
### Check topic list 
In a new terminal check the list of topics.  

    ros2 topic list
### Echo topic 
    ros2 topic echo /rp2040_imu_info_topic
  
Note: If topic is not visible check value of ROS_DOMAIN_ID in bashrc since micro ros uses ROS_DOMAIN_ID = 0. 

Resources:  
[1] https://adityakamath.github.io/2022-06-19-microros-examples/  
[2] https://docs.arduino.cc/tutorials/nano-rp2040-connect/rp2040-imu-basics  
      
