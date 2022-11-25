/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:56:46
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 16:54:04
 */

#include "glog/logging.h"
#include "mapping/mapping/mapping_flow.hpp"
#include "ros/ros.h"

using namespace avp_mapping;

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_mapping_node");
  ros::NodeHandle nh;
  std::string work_dir;
  nh.param<std::string>("work_dir", work_dir, "");
  if (work_dir == "") {
    LOG(ERROR) << "Can not find work_dir ";
    return -1;
  }

  std::shared_ptr<MappingFlow> mapping_flow_ptr =
      std::make_shared<MappingFlow>(nh, work_dir);

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    mapping_flow_ptr->run();
    r.sleep();
  }

  return 0;
}
