/*
 * @Author: ding.yin
 * @Date: 2022-10-02 14:23:54
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 21:24:22
 */
#ifndef _DATA_PRETREAT_FLOW_H_
#define _DATA_PRETREAT_FLOW_H_

#include <memory>

#include <ros/ros.h>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"
#include "tf_listener/tf_listener.hpp"

#include "publisher/cloud_publisher.hpp"
#include "publisher/tf_broadcaster.hpp"

namespace avp_mapping {

class DataPretreatFlow {

public:
  DataPretreatFlow(ros::NodeHandle &nh);

  bool run();

private:
  bool readData();
  bool initCalibration();
  bool hasData();
  bool validData();
  bool publishData();

private:
  // subscriber
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr1_;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr2_;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr3_;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr4_;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr5_;
  std::shared_ptr<CloudSubscriber> cloud_sub_ptr6_;
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<TFBroadCaster> tfbroad_pub_ptr_;

  std::deque<CloudData> cloud1_data_buff_;
  std::deque<CloudData> cloud2_data_buff_;
  std::deque<CloudData> cloud3_data_buff_;
  std::deque<CloudData> cloud4_data_buff_;
  std::deque<CloudData> cloud5_data_buff_;
  std::deque<CloudData> cloud6_data_buff_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<PoseData> odom_data_buff_;

  CloudData cur_cloud_1_;
  CloudData cur_cloud_2_;
  CloudData cur_cloud_3_;
  CloudData cur_cloud_4_;
  CloudData cur_cloud_5_;
  CloudData cur_cloud_6_;
  IMUData cur_imu_;
  PoseData cur_odom_;
};

} // namespace avp_mapping

#endif // _DATA_PRETREAT_FLOW_H_
