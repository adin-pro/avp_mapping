/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:47:58
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 20:55:12
 */

#include "mapping/loop_close/loop_closing_flow.hpp"
#include "ros/ros.h"

using namespace avp_mapping;

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_loop_closing_node");
  ros::NodeHandle nh;
  std::string work_dir = "/home/adin/catkin_ws/src/avp_mapping/";
  std::shared_ptr<LoopClosingFlow> lpf_ptr =
      std::make_shared<LoopClosingFlow>(nh, work_dir);

  ros::Rate r(100);
  while (ros::ok()) {
    ros::spinOnce();
    lpf_ptr->run();
    r.sleep();
  }

  return 0;
}
