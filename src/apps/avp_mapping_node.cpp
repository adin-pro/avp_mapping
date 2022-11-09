/*
 * @Author: ding.yin 
 * @Date: 2022-11-09 20:56:46 
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 21:05:27
 */

#include "mapping/mapping/mapping_flow.hpp"
#include "ros/ros.h"

using namespace avp_mapping;

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_mapping_node");
  ros::NodeHandle nh;
  std::string work_dir = "/home/adin/catkin_ws/src/avp_mapping/";
  
  std::shared_ptr<MappingFlow> mapping_flow_ptr = std::make_shared<MappingFlow>(nh);

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    r.sleep();
  }

  return 0;
}


