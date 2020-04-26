#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>
#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

ros::Publisher botCmd;

bool handle_command(ball_chaser::DriveToTarget::Request& req,
                    ball_chaser::DriveToTarget::Response& res) {
  // ROS_INFO("DriveToTarget request received - linear_x:%1.2f, angular_z:%1.2f",
  //          (float)req.linear_x, (float)req.angular_z);
  geometry_msgs::Twist cmd;
  cmd.linear.x = (float)req.linear_x;
  cmd.linear.y = 0.0;
  cmd.linear.z = 0.0;

  cmd.angular.x = 0.0;
  cmd.angular.y = 0.0;
  cmd.angular.z = (float)req.angular_z;

  botCmd.publish(cmd);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle n;

  ros::ServiceServer service =
      n.advertiseService("/ball_chaser/command_robot", handle_command);

  botCmd = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
  ROS_INFO("Ready to send robot commands");

  ros::spin();

  return 0;
}
