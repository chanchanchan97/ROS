# ROS
ROS communication codes and Gazebo simulation

1. 运行环境：Ubuntu16.04 + ROS Kinetic + Gazebo8.0
2. 运行步骤：
 -  Stanley算法launch文件运行步骤：  
  * -> smartcar_description/smartcar_gazebo.launch 
  * -> waypoint_loader/waypoint_loader.launch 
  * -> waypoint_updater/waypoint_updater.launch 
  * -> stanley_persuit/stanley_persuit.launch
 - Pure_persuit算法launch文件运行步骤：  
  * -> smartcar_description/smartcar_gazebo.launch 
  * -> waypoint_loader/waypoint_loader.launch 
  * -> waypoint_updater/waypoint_updater.launch 
  * -> pure_persuit/pure_persuit.launch
 - Haar特征车辆识别launch文件运行步骤：  
  * -> robot_vision/vehicle_detector.launch
