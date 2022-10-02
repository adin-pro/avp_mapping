/*
 * @Author: ding.yin 
 * @Date: 2022-10-02 14:23:54 
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 21:19:15
 */
#ifndef _DATA_PRETREAT_FLOW_H_
#define _DATA_PRETREAT_FLOW_H_

#include <ros/ros.h>


namespace avp_mapping {

class DataPretreatFlow {

 public:
  DataPretreatFlow(ros::NodeHandle& nh);
 
  bool run();

 private:
  bool readData();
  bool initCalibration();
  bool hasData();
  bool validData();
  bool publishData();



};


} // avp_mapping

#endif // _DATA_PRETREAT_FLOW_H_
