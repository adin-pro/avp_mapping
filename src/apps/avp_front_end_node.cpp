/*
 * @Author: ding.yin
 * @Date: 2022-11-04 15:15:03
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-05 21:32:12
 */

#include "mapping/front_end/front_end_flow.hpp"

using namespace avp_mapping;

int main(int argc, char **argv) {

  ros::init(argc, argv, "avp_front_end");
  ros::NodeHandle nh;

  std::string cloud_topic = "/bev/rgb_cloud";
  std::string odom_pub_topic = "/vidar_odom";

  std::shared_ptr<FrontEndFlow> front_end_ptr =
      std::make_shared<FrontEndFlow>(nh, cloud_topic, odom_pub_topic);

  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    front_end_ptr->run();
    rate.sleep();
  }

  return 0;
}
