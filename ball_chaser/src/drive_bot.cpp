#include <std_msgs/Float64.h>
#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

bool handle_command(ball_chaser::DriveToTarget::Request& req,
                    ball_chaser::DriveToTarget::Response& res) {
  ROS_INFO("DriveToTarget request received - linear_x:%1.2f, angular_z:%1.2f",
           (float)req.linear_x, (float)req.angular_z);
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "drive_bot");
  ros::NodeHandle n;

  ros::ServiceServer service =
      n.advertiseService("/ball_chaser/command_robot", handle_command);

  ROS_INFO("Ready to send robot commands");

  ros::spin();

  return 0;
}
