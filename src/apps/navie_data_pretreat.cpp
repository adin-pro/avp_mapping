/*
 * @Author: ding.yin 
 * @Date: 2022-11-03 11:52:32 
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-03 15:58:17
 */

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "glog/logging.h"
#include <ros/ros.h>

#include "data_pretreat/data_pretreat_flow.hpp"
#include "global_defination/global_defination.h.in"

using namespace avp_mapping;


void exit_logger(int s) {
  std::cout <<  "\ncatch ctrl^C, system will exit" << std::endl;
  _exit(1);
}

int main(int argc, char **argv) {
  signal(SIGINT, exit_logger);
  google::InitGoogleLogging(argv[0]);
  FLAGS_alsologtostderr = true;

  ros::init(argc, argv, "navie_data_pretreat");
  ros::NodeHandle nh;
  ros::Rate rate(100);

  std::shared_ptr<DataPretreatFlow> data_pretreat_flow_ptr =
      std::make_shared<DataPretreatFlow>(nh);

  while (ros::ok()) {
    ros::spinOnce();
    data_pretreat_flow_ptr->run();
    // LOG(INFO) << "flow main";
    rate.sleep();
  }

  return 0;
}



