/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:57:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 21:14:20
 */

#ifndef _MAPPING_FLOW_H_
#define _MAPPING_FLOW_H_

#include <deque>

#include "sensor_data/pose_data.hpp"

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "publisher/cloud_publisher.hpp"

#include "mapping/mapping/mapping_core.hpp"

#include "ros/ros.h"

namespace avp_mapping {
class MappingFlow {
public:
  MappingFlow(ros::NodeHandle &nh);

  bool run();

private:
  bool readData();
  bool validData();
  bool hasData();
  bool transformData();
  bool publishData();

private:
  std::deque<CloudData> cloud_data_buff_;
  std::deque<PoseData> odom_data_buff_;

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr_;
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;

  std::shared_ptr<MappingCore> mapping_core_ptr_;

  CloudData cuur_cloud_data_;
  PoseData curr_odom_;
};

} // namespace avp_mapping

#endif // _MAPPING_FLOW_H_
