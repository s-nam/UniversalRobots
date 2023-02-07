# Linking a Robotiq 2F gripper to UR10 for Moveit

## 1. Conditions
- OS: Ubuntu 20.04.5 LTS 64-bit
- Robot: UR10 (CB series)
- Gripper: Robotiq 2F-85
- Moveit: Moveit Noetic

## 2. Background and purpose
After you successfully attached a gripper to the tip of the robot arm in the physical (real) world, you probably want to create the exactly same simulating environment in the virtual world, such as Moveit or Gazebo.

To do that, you need to combine the URDF files from both the gripper and robot arm. However, there is a little information on how to combine them together. Some infomation was definitely helpful, but most of them were based on old version.

## 3. Prerequisite
<!--- You first need to have a URDF (or xacro) file for UR10. The ``Universal Robots ROS Driver`` contains the file. Please check [here](https://github.com/s-nam/UniversalRobots/blob/main/Installation_guides/01_UR10_on_ROS-noetic/README.md#4-install-universal-robots-ros-driver) for the installation instructions. --->

You first need to have a URDF (or xacro) file for UR10. The ``ros-industrial/universal_robot`` contains the file. Please check [here](https://github.com/ros-industrial/universal_robot) for the installation instructions.

Second, you need to have a URDF (or xacro) file for Robotiq 2F-85. I found a well-maintained repository for the gripper from [here](https://github.com/TAMS-Group/robotiq). Please clone the repository under ``catkin_ws/src``.

## 4. Helpful sites
 - [URDF tutorials](http://wiki.ros.org/urdf/Tutorials)
    > I strongly recommend to read through ``2. Learning URDF Step by Step`` section before following my guidelines.
    
 - [Moveit config for the UR5 and a gripper](https://roboticscasual.com/ros-tutorial-how-to-create-a-moveit-config-for-the-ur5-and-a-gripper/)
    > Although the descriptions were written based on an old-fashioned way, which is not valid on ROS Noetic, it is worth reading.

 - [MultiCobot-UR10-Gripper from Serru]https://serru.github.io/MultiCobot-UR10-Gripper/) and its [repository](https://github.com/Serru/MultiCobot-UR10-Gripper)

## 5. Steps
### 5.1. Create a catkin package

```consol
$ cd ~/catkin_ws/src
$ catkin_create_pkg cobot_test2 std_msgs rospy roscpp
```

And create a folder called ``cobot_description`` under ``catkin_ws/src/cobot_test2``.

```consol
$ cd cobot_test2
$ mkdir cobot_description
```

### 5.2. Create a xacro file

```consol
$ cd cobot_description
$ touch cobot_test2.xacro
```

Open the xacro file with an editor and paste the following codes.

```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="ur10_robotiq2f">

  <!-- common stuff -->
  <!-- <xacro:include filename="$(find ur_gazebo)/urdf/ur.xacro" /> -->

  <!-- include macros for UR10 and Robotiq 2f hand -->
  <xacro:include filename="$(find ur_description)/urdf/inc/ur10_macro.xacro" />
  <xacro:include filename="$(find robotiq_2f_85_gripper_visualization)/urdf/robotiq_arg2f_85_model_macro.xacro" />

  <!-- create the robot + eef combo itself as a macro -->
  <xacro:macro name="ur10_with_robotiq" params="prefix">

    <!-- instantiate UR10 and Robotiq 2f hand -->
    <xacro:ur10_robot prefix="${prefix}robot_"/>
    <xacro:robotiq_arg2f_85 prefix="${prefix}gripper_" />

    <!-- attach gripper to UR10 -->
    <joint name="${prefix}tool0-${prefix}robotiq_arg2f_base_link" type="fixed">
      <origin xyz="0 0 0" rpy="0 0 0" />
      <parent link="${prefix}robot_tool0" />
      <child link="${prefix}gripper_robotiq_arg2f_base_link" />
    </joint>

    <!-- Define th ur10's position and orientation in the world coordinate system -->
    <link name="world" />
    <joint name="world_joint" type="fixed">
        <parent link="world" />
        <child link="${prefix}robot_base_link" />
        <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0" />
    </joint>
  </xacro:macro>

<xacro:ur10_with_robotiq prefix=""/>


</robot>

```

### 5.3. Detailed code explanation
- ``find ur_description``
    > The Linux system automatically finds the path of a folder called "ur_description". In my case, it is located ``catkin_ws/src/universal_robot``. In addition, you should check if you can find ``ur10_macro.xacro`` file under ``./urdf/inc``.

- Basically, ``ur10_macro.xacro`` contains all necessary information for UR10 and does ``robotiq_arg2f_85_model_macro.xacro`` for Robotiq 2F gripper.

- The codes below `` <!-- create the robot + eef combo itself as a macro -->``
    > They are working as a glue connecting UR10 with Robotiq 2F.


## 6. Check the scrutiny

First, you need to convert from xacro to urdf file. Under ``catkin_ws/src/cobot_test/cobot_description``

```console
$ xacro cobot_test2.xacro > cobot_test2.urdf
```

Check the urdf file. You should not see any errors in this step.

```console
$ check_urdf cobot_test2.urdf
robot name is: ur10_robotiq2f
---------- Successfully Parsed XML ---------------
root Link: world has 1 child(ren)
    child(1):  robot_base_link
        child(1):  robot_base
        child(2):  robot_base_link_inertia
            child(1):  robot_shoulder_link
                child(1):  robot_upper_arm_link
                    child(1):  robot_forearm_link
                        child(1):  robot_wrist_1_link
                            child(1):  robot_wrist_2_link
                                child(1):  robot_wrist_3_link
                                    child(1):  robot_flange
                                        child(1):  robot_tool0
                                            child(1):  gripper_robotiq_arg2f_base_link
                                                child(1):  gripper_left_outer_knuckle
                                                    child(1):  gripper_left_outer_finger
                                                        child(1):  gripper_left_inner_finger
                                                            child(1):  gripper_left_inner_finger_pad
                                                child(2):  gripper_left_inner_knuckle
                                                child(3):  gripper_right_inner_knuckle
                                                child(4):  gripper_right_outer_knuckle
                                                    child(1):  gripper_right_outer_finger
                                                        child(1):  gripper_right_inner_finger
                                                            child(1):  gripper_right_inner_finger_pad
```
