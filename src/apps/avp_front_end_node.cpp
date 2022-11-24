/*
 * @Author: ding.yin
 * @Date: 2022-11-04 15:15:03
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-05 21:32:12
 */

#include "mapping/front_end/front_end_flow.hpp"

#include "glog/logging.h"

using namespace avp_mapping;

int main(int argc, char **argv) {

  ros::init(argc, argv, "avp_front_end");
  ros::NodeHandle nh;

  std::string cloud_topic = "/bev/rgb_cloud";
  std::string odom_pub_topic = "/vidar_odom";

  std::string work_dir;
  nh.param<std::string>("work_dir", work_dir, "");
  if (work_dir == "") {
    LOG(ERROR) << "Can not find work_dir ";
    return -1;
  }

  std::shared_ptr<FrontEndFlow> front_end_ptr =
      std::make_shared<FrontEndFlow>(nh, work_dir);

  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    front_end_ptr->run();
    rate.sleep();
  }

  return 0;
}
