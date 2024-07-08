# Manipulate-with-turtlesim-package-in-ROS-noetic-and-ROS-foxy

### Ros noetic : The turtlesim package in ROS Noetic is a simple simulator designed to help users learn ROS (Robot Operating System) concepts. It provides a graphical interface with a turtle that can be moved around using ROS commands, topics, and services. This package is excellent for beginners to get hands-on experience with ROS without needing physical hardware.

## 1- Install Turtlesim : If you haven't installed turtlesim, you can do so with the following command:
```
sudo apt-get install ros-noetic-turtlesim

```

## 2- Launch Turtlesim 

Start the ROS master node:

```
roscore
```
Open another terminal window and launch the turtlesim node:

```
rosrun turtlesim turtlesim_node

```
## 3- Move the Turtle
```
rosrun turtlesim turtle_teleop_key

```

## 4- Using ROS Topics
```
rostopic pub /turtle1/cmd_vel geometry_msgs/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 1.8"

```

## 5- Writing a ROS Node to Control the Turtle

1- Create a workspace if you don't have one:
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
catkin_make
source devel/setup.bash
```

2- Create a package:
```
cd src
catkin_create_pkg my_turtle_control rospy std_msgs geometry_msgs
cd ..
catkin_make
source devel/setup.bash

```
3- Create a Python script to control the turtle:
```
cd src/my_turtle_control
mkdir scripts
cd scripts
```

4-Create a file named move_turtle.py and add the following code:
```
#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def move_turtle():
    rospy.init_node('move_turtle', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1)  # 1 Hz

    while not rospy.is_shutdown():
        vel_msg = Twist()

        vel_msg.linear.x = 2.0
        vel_msg.angular.z = 1.8

        rospy.loginfo("Moving the turtle")
        pub.publish(vel_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        move_turtle()
    except rospy.ROSInterruptException:
        pass

```

5-Make the script executable:
```
chmod +x move_turtle.py

```
6- Run the script:
```
rosrun my_turtle_control move_turtle.py

```
