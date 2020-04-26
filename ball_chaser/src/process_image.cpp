#include <sensor_msgs/Image.h>
#include <std_msgs/Float64.h>
#include "ball_chaser/DriveToTarget.h"
#include "ros/ros.h"

class SubscribeAndCallService {
 public:
  SubscribeAndCallService() {
    client_ = n_.serviceClient<ball_chaser::DriveToTarget>(
        "/ball_chaser/command_robot");
    sub_ = n_.subscribe("/camera/rgb/image_raw", 10,
                        &SubscribeAndCallService::imgCallback, this);

    pub_ = n_.advertise<sensor_msgs::Image>("/ball_chaser/processed_image", 10);
    // ball_chaser::DriveToTarget srv;
    // srv.request.angular_z = 0.5;
    // for (int i = 0; i < 5; i++) {
    //   ros::Duration(5).sleep();
    //   srv.request.linear_x = i;

    //   if (!client_.call(srv)) {
    //     ROS_INFO("Error calling service");
    //   }
    // }
  }

  void imgCallback(const sensor_msgs::Image img) {
    uint32_t sumCol = 0, sumRow = 0, area = 0;

    for (uint32_t i = 0; i < img.height * img.step; i++) {
      if (img.data[i] == 255) {
        sumRow += i / img.step;
        sumCol += i % img.step;
        area += 1;
      }
    }

    if (area > 0) {
      ROS_INFO("row_center: %d, col_center: %d, area: %d", sumRow / area,
               sumCol / area, area);
    }
  }

 private:
  ros::NodeHandle n_;
  ros::Subscriber sub_;
  ros::Publisher pub_;
  ros::ServiceClient client_;
};

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "process_image");

  SubscribeAndCallService SASObj;

  ros::spin();

  return 0;
}
