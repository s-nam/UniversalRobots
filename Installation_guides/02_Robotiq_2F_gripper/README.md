# Install Robotiq 2F gripper on UR robot

## 1. Conditions
- OS: Ubuntu 20.04.5 LTS 64-bit
- Robot: UR10 (CB series)
- Polyscope version: 3.15.8
- For CB series, you should connect the USB cord of the gripper to the remote PC, rather than to the control box.

    > If you are using e-series, it seems possible to connect the USB to the control box and to control the gripper from a remote PC. Please refer to [this](https://answers.ros.org/question/399617/how-to-control-a-robotiq-2f-85-in-ros-via-the-ur-tool-io/) and [this](https://github.com/UniversalRobots/Universal_Robots_ToolComm_Forwarder_URCap).

## 2. Installation
Please follow [this](http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29).

When you get an queue_size error, please refer to [this](http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers).

```Python
# The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pub = rospy.Publisher(
        "Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, queue_size=10
    )
```