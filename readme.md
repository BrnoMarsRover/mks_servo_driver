# How to install

### Install ros2:

https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html


### Install ros control:
```
sudo apt install ros-humble-ros2-control
```

### Install ros topic based control:
https://github.com/PickNikRobotics/topic_based_ros2_control/tree/main?tab=readme-ov-file
```
sudo apt install ros-humble-topic-based-ros2-control
```

### Install python dependencies:
```
pip3 install python-can pyserial numpy
```

### Navigate to folder /src:
```
sudo rosdep init
```
```
sudo rosdep update
```
```
sudo rosdep install --from-paths ./ -i -y --rosdistro ${ROS_DISTRO}
```
# How to integrate robot
Navigate to the control ros2 control .xacro file and in the \<hardware> section include topic based plugin together with topic definition for state and command:
```
<plugin>topic_based_ros2_control/TopicBasedSystem</plugin>
<param name="joint_commands_topic">/manipulator_joints_cmd</param>
<param name="joint_states_topic">/manipulator_joints_state</param>
```
Topics use sensor_msgs/JointState.msg format

# How to configure driver
In src/python_driver/python_driver/driver.py file specify joint count of your robot by modifying JOINT_CNT variable. Interfaces with servos are created automatically with IDs in ascending order and starting with ID 1.

#### Alternatively you can specify interfaces manually by replacing:
```
self.servos = [ MksServo(self.bus, self.notifier, i, self.HOMING_SPEEDS[i - 1]) for i in range(1, self.JOINT_CNT + 1) ]
```
with:
```
self.servos = [
    MksServo(self.bus, self.notifier, 1, self.HOMING_SPEEDS[1]),
    MksServo(self.bus, self.notifier, 5, self.HOMING_SPEEDS[5])
]
```
where the second to last argument specifies servo CAN ID.

#### It is also important to specify can channel by changing *channel* argument:
```
self.bus = can.interface.Bus(interface='socketcan', channel='can0', bitrate=125000)
```

# How to build
In workspace root
```
colcon build
```
```
source install/setup.bash
```

# How to run
```
ros2 run manipulator_servo_driver mks_interface 
```

# How to use
~~By default, the driver is in standby mode (mode 0). In this mode joint states are published and all commands are ignored.~~

Set to position mode by default for easier development

Mode can be changed by calling mks_servo_change_mode service:
```
ros2 service call /mks_servo_change_mode python_driver_interfaces/srv/ChangeMode "{mode: 1}"
```
Don't forget to source the workspace before running the command.

#### Driver supports 3 different modes:
- 0 - Standby (joint states are published, command are ignored)
- 1 - Position (joint states are published, position commands are used) 
- 2 - Speed (joint states are published, velocity commands are used - currently very dangerous with the hardware. In the future, it would be a good idea to test velocity mode with a separate motor)  

#### Driver also allows user to reset the servo axis to 0 by calling:
```
ros2 service call /mks_servo_reset_axis python_driver_interfaces/srv/ResetAxis
```


# How to run test demo
This demo works with at least 2 servos connected i.e. JOINT_CNT in driver should be 2.
```
ros2 launch ros2_control_demo_example_1 rrbot.launch.py 
```

# Limitations
During testing we have observed that servos continue to run even when the communication is lost.
