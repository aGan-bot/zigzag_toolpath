# Ros2 Humble
Zigzag_toolpath

## 📌 Description
making zigzag toolpath around a frame

## 🚀 ROS2 Distribution
Tested on: ROS2 Humble

## 📦 Dependencies
- rclpy
- geometry_msgs
- trajectory_msgs
- moveit_msgs

## 🔧 Installation
```bash
cd ~/ros2_ws/src
git clone https://github.com/aGan-bot/zigzag_toolpath.git
cd ..
colcon build
source install/setup.bash
```

## Use
Sure about ros2 control command interface is position in fr3_ros_controllers.yaml

Terminal 1
```bash
 ros2 launch franka_fr3_moveit_config moveit.launch.py robot_ip:=dont-care use_fake_hardware:=true fake_sensor_commands:=true
```
Terminal 2
```bash
ros2 launch zigzag_toolpath zigzag.launch.py
```
