/*
 * @Author: ding.yin
 * @Date: 2022-10-02 14:24:27
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:23:47
 */

#include "glog/logging.h"
#include <ros/ros.h>

#include "global_defination/global_defination.h"

using namespace avp_mapping;

int main(int argc, char **argv) {
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "avp_data_pretreat_node");
  ros::NodeHandle nh;
  ros::Rate rate(100);

  while (ros::ok()) {
    ros::spinOnce();
    LOG(INFO) << 1;
    rate.sleep();
  }

  return 0;
}
