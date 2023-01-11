# Install Robotiq 2F gripper on UR robot

## 1. Conditions
- OS: Ubuntu 20.04.5 LTS 64-bit
- Robot: UR10 (CB series)
- Polyscope version: 3.15.8
- For CB series, you should **connect the USB cord** of the gripper **to the remote PC**, rather than to the control box.

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

## 3. Running the gripper
First of all, please turn on the power supply to give 24 VDC (Set the maximum voltage as 24V and the maximum current as 1.0 A, and volatge control mode), and check if you can see either blue or led light on the gripper.

Open a terminal and run
```console
$ roscore
```

Open another terminal and run
```console
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```

Open another terminal and run `rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py`

If you see the `queue_size` error message, such as below:
```console
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

/home/softtas2022/catkin_ws/src/robotiq/robotiq_2f_gripper_control/nodes/Robotiq2FGripperSimpleController.py:135: SyntaxWarning: The publisher should be created with an explicit keyword argument 'queue_size'. Please see http://wiki.ros.org/rospy/Overview/Publishers%20and%20Subscribers for more information.
  pub = rospy.Publisher(
Simple 2F Gripper Controller
-----
Current command:  rACT = 0, rGTO = 0, rATR = 0, rPR = 0, rSP = 0, rFR = 0
-----
Available commands

r: Reset
a: Activate
c: Close
o: Open
(0-255): Go to that position
f: Faster
l: Slower
i: Increase force
d: Decrease force
-->
```
In this case, please open `Robotiq2FGripperSimpleController.py` in `~/catkin_ws/src/robotiq/robotiq_2f_gripper_control/nodes/` and add `queue_size=10`, such as below

```Python
...
# The Gripper status is published on the topic named 'Robotiq2FGripperRobotInput'
    pub = rospy.Publisher(
        "Robotiq2FGripperRobotInput", inputMsg.Robotiq2FGripper_robot_input, queue_size=10
    )
...
```

## 4. Issue - gFLT = 14: Overcurrent protection triggered

The gripper's listener provides the gripper's status.

```console
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperStatusListener.py
```

Whenever the grippers light blinks in blue and red colors, check the status using the listener.

I found **"gFLT = 14: Overcurrent protection triggered"** whenever I activated the gripper.
I guess the current gets over the limit internally defined if

- the gripper wants to reach the 0 gap even if our gripper hardly grips to 0. (Overcurrent)
- the gripper want to reach the 255 gap even if our gripper hardly grips to 255. (Overcurrent)
- the gripper could not produce higher current when closing it. (Undercurrent)

### 4.1. Overcurrent case
My solution is to set the gripper work within 10 - 250. In addition, decrease the force and speed.

### 4.2. Undercurrent case
Set the maximum current as **1.2 A** in the DC power supply (although the official manual indicates 1.0 A), and the maximum voltage as 24 V.

## 5. Simple Python code to control the gripper
Please see [this repository](https://github.com/s-nam/control_2f_gripper_and_ur10).