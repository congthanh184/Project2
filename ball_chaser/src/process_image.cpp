#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

enum class BotState { REST, ROTATING, RUNNING };

class SubscribeAndCallService {
 public:
  static const uint16_t LEFT = 1000;
  static const uint16_t RIGHT = 1600;
  static const uint32_t STOP_THRESHOLD = 150000;

  SubscribeAndCallService() {
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>(
        "/ball_chaser/command_robot");
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10,
                        &SubscribeAndCallService::imgCallback, this);

    state_ = BotState::REST;
    ros::Duration(0.5).sleep();
    brake();
  }

  void callDriveToTarget(const double linear_x, const double angular_z) {
    ball_chaser::DriveToTarget srv;
    srv.request.angular_z = angular_z;
    srv.request.linear_x = linear_x;

    if (!client_.call(srv)) {
      ROS_INFO("Error calling service");
    }
  }

  void rotateLeft() {
    ROS_INFO("GO LEFT");
    callDriveToTarget(0.0, 0.2);
  }

  void rotateRight() {
    ROS_INFO("GO RIGHT");
    callDriveToTarget(0.0, -0.2);
  }

  void goStraight() {
    ROS_INFO("GO STRAIGHT");
    callDriveToTarget(0.2, 0.0);
  }

  void brake() {
    ROS_INFO("BRAKE");
    callDriveToTarget(0.0, 0.0);
  }

  void imgCallback(const sensor_msgs::Image img) {
    uint32_t sumCol = 0, sumRow = 0, area = 0;
    uint32_t centerCol;

    for (uint32_t i = 0; i < img.height * img.step; i++) {
      if (img.data[i] == 255) {
        sumRow += i / img.step;
        sumCol += i % img.step;
        area += 1;
      }
    }

    if (area > 0 && area < STOP_THRESHOLD) {
      centerCol = sumCol / area;

      ROS_INFO("row_center: %d, col_center: %d, area: %d", sumRow / area,
               centerCol, area);

      if (centerCol < LEFT) {
        rotateLeft();
      } else if (centerCol > RIGHT) {
        rotateRight();
      } else {
        goStraight();
      }
    } else {
      brake();
    }
  }

 private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
  BotState state_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "process_image");

  SubscribeAndCallService SASObj;

  ros::spin();

  return 0;
}
