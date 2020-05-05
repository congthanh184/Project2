# go-chase-it

A controller that can detect and drive robot to the white ball with ROS and Gazebo.

## How to 

- Clone to `~/catkin_ws/src`
- `cd ~/catkin_ws`
- `catkin_make`
- `source devel/setup.bash`
- `roslaunch my_robot world.launch`
- On a another terminal, `roslaunch ball_chaser ball_chaser.launch`

Move the white ball around the map (in Gazebo) to see how robot reacts and follows it.

## What's next

Implement openCV library for more advanced features
