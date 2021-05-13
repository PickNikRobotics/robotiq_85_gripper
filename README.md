# robotiq_85_gripper
Common packages for the Robotiq 85 Gripper provided by Stanley Innovation


# Build 
```
colcon build --symlink-install
```

# Run 
By defualt the driver will open up port 'ttyUSB0' and 115200 baud rate.

```
ros2 launch robotiq_85_driver gripper_driver.launch.py
```

Open/close the gripper by sending an action request. The position is the distance between the end effectors (max is 8cm)
```
ros2 action send_goal /gripper_action/GripperCommand control_msgs/action/GripperCommand "{command: {position: 0.05, max_effort: 5.0}}"
```

# Communication Port  
There have been issues with opening a comm port for the gripper. As a debugging step make set the port permissions and add the user to the dialout group. 
```
sudo chmod 666 /dev/ttyUSB0
sudo chown $USER /dev/ttyUSB0
sudo adduser $USER dialout
```

# Issues 
The ros2 driver does not currently work with MoveIt2. When the action server sends a gripper command to either open or close the gripper the worng joint states are pubished by the driver causing MoveIt2 to report a failure. 

