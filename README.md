# afrl_radar_ros

## Platform: Ubutnu 16.04 with ROS Kinetic

## Requirements:

- radar_msgs

      sudo apt install ros-kinetic-radar-msgs
      
- glog

      sudo apt install libgoogle-glog-dev
      
- atlas-base
      
      sudo apt install libatlas-base-dev
      
      
 ## CAN driver and rules file:
      cd dependencies
      ./install.sh


 ## To Use:
      
      roslaunch afrl_radar_ros afrl_radar.launch
