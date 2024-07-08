# Manipulate-with-turtlesim-package-in-ROS-noetic-and-ROS-foxy
## ROS 1
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

## ROS 2 
### ROS foxy : Turtlesim is a ROS2 package used for learning and demonstrating basic robotics concepts. It provides a simple, graphical interface where users can control a turtle in a 2D space, allowing them to understand topics like publishing and subscribing to topics, and working with ROS2 nodes.

## 1- Using Turtlesim

Source the ROS2 setup script:
```
source /opt/ros/foxy/setup.bash
```

Launch Turtlesim Node:
```
ros2 run turtlesim turtlesim_node
```

## 2-Using Teleop
```
source /opt/ros/foxy/setup.bash
ros2 run turtlesim turtle_teleop_key
```

## 3-Publishing Messages

```
source /opt/ros/foxy/setup.bash
ros2 topic pub /turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.8}}"
```

## 4- Writing a Simple Node to Control Turtlesim

1- Create a Workspace:

```
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
colcon build
source install/setup.bash
```

2- Create a Package:

```
cd src
ros2 pkg create --build-type ament_python my_turtle_controller
```

3- Create a Python Script:

```
cd my_turtle_controller/my_turtle_controller
touch turtle_controller.py
chmod +x turtle_controller.py
```

4-Edit the Script:

```
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class TurtleController(Node):
    def __init__(self):
        super().__init__('turtle_controller')
        self.publisher_ = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        self.timer = self.create_timer(1.0, self.move_turtle)
        self.count = 0

    def move_turtle(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.8
        self.publisher_.publish(msg)
        self.count += 1
        if self.count > 10:
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    turtle_controller = TurtleController()
    rclpy.spin(turtle_controller)
    turtle_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

5- Modify setup.py:

```
entry_points={
    'console_scripts': [
        'turtle_controller = my_turtle_controller.turtle_controller:main',
    ],
},
```

6-Build the Package:

```
cd ~/ros2_ws
colcon build
source install/setup.bash
```
7-Run the Script:

```
ros2 run my_turtle_controller turtle_controller

```

