/*
 * @Author: ding.yin
 * @Date: 2022-11-07 21:48:00
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-08 20:38:56
 */

#include "glog/logging.h"
#include "mapping/back_end/back_end_flow.hpp"

using namespace avp_mapping;

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_back_end_node");
  ros::NodeHandle nh;
  ros::Rate r(100);

  std::string work_dir;
  nh.param<std::string>("work_dir", work_dir, "");
  if (work_dir == "") {
    LOG(ERROR) << "Can not find work_dir ";
    return -1;
  }

  std::shared_ptr<BackEndFlow> back_end_flow_ptr_ =
      std::make_shared<BackEndFlow>(nh, work_dir);

  while (ros::ok()) {
    ros::spinOnce();
    back_end_flow_ptr_->run();
    r.sleep();
  }
  return 0;
}