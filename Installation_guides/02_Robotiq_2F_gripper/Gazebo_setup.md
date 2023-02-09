# Setup Robotiq 2F gripper and UR10 for Gazeo simulation

## 1. Install Gazebo for ROS Noetic
    https://classic.gazebosim.org/tutorials?tut=ros_installing&cat=connect_ros


In case you see errors after ``catkin_make``, such as
```console
/usr/bin/ld: cannot find -lDepthCameraPlugin
/usr/bin/ld: cannot find -lGpuRayPlugin
/usr/bin/ld: cannot find -lContactPlugin
/usr/bin/ld: cannot find -lCameraPlugin
/usr/bin/ld: cannot find -lRayPlugin
collect2: error: ld returned 1 exit status
make[2]: *** [robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/CMakeFiles/RobotiqHandPlugin.dir/build.make:338: /home/snam/catkin_ws/devel/lib/libRobotiqHandPlugin.so] Error 1
make[1]: *** [CMakeFiles/Makefile2:22663: robotiq/robotiq_3f_gripper_articulated_gazebo_plugins/CMakeFiles/RobotiqHandPlugin.dir/all] Error 2
make: *** [Makefile:141: all] Error 2
Invoking "make -j6 -l6" failed
```
My solution is to set problematic folders ``ignore`` to build, because such the folders (currently ``robotiq_3f_gripper_articulated_gazebo_plugins``) is not used.

```console
~/catkin_ws/src/robotiq/robotiq_3f_gripper_articulated_gazebo_plugins$ touch CATKIN_IGNORE

# I additionally ignored robotiq_3f_gripper_articulated_gazebo folder.

~/catkin_ws/src/robotiq/robotiq_3f_gripper_articulated_gazebo$ touch CATKIN_IGNORE
```