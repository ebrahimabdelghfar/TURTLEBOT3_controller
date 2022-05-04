# TURTLEBOT3_controller
ASUFE course project used turtlebot3 simulation
# Dependencies 
```
numpy #version 1.17.4
```
# How to run
in first terminal </br>
```
source devel/setup.bash
export TURTLEBOT3_MODEL=waffle_pi
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch 
```
open seconed terminal in same workspace
```
source devel/setup.bash
rosrun milestones_code Turtle_Control.py
```
# Video
the following video show working go-to controller </br>


https://user-images.githubusercontent.com/81301684/166640629-58b60371-94b7-4d10-8281-04df76f7a6a4.mp4




