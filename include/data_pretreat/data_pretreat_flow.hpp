/*
 * @Author: ding.yin
 * @Date: 2022-10-02 14:23:54
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:40:59
 */
#ifndef _DATA_PRETREAT_FLOW_H_
#define _DATA_PRETREAT_FLOW_H_

#include <memory>

#include <ros/ros.h>

#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/image_subscriber.hpp"
#include "subscriber/imu_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"

#include "tf_listener/tf_listener.hpp"

#include "publisher/cloud_publisher.hpp"
#include "publisher/image_publisher.hpp"
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
  bool transformData();

private:
  // subscriber
  std::shared_ptr<ImageSubscriber> img_sub_ptr_0_;
  std::shared_ptr<ImageSubscriber> img_sub_ptr_1_;
  std::shared_ptr<ImageSubscriber> img_sub_ptr_2_;
  std::shared_ptr<ImageSubscriber> img_sub_ptr_3_;
  std::shared_ptr<ImageSubscriber> img_sub_ptr_4_;
  std::shared_ptr<ImageSubscriber> img_sub_ptr_5_;
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr_;
  std::shared_ptr<IMUSubscriber> imu_sub_ptr_;
  // tf listener
  std::shared_ptr<TFListener> camera0_to_base_ptr_;
  std::shared_ptr<TFListener> camera1_to_base_ptr_;
  std::shared_ptr<TFListener> camera2_to_base_ptr_;
  std::shared_ptr<TFListener> camera3_to_base_ptr_;
  std::shared_ptr<TFListener> camera4_to_base_ptr_;
  std::shared_ptr<TFListener> camera5_to_base_ptr_;

  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<ImagePublisher> img_pub_ptr_;
  std::shared_ptr<TFBroadCaster> tfbroad_pub_ptr_;

  std::deque<ImageData> img_data_buff_0_;
  std::deque<ImageData> img_data_buff_1_;
  std::deque<ImageData> img_data_buff_2_;
  std::deque<ImageData> img_data_buff_3_;
  std::deque<ImageData> img_data_buff_4_;
  std::deque<ImageData> img_data_buff_5_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<PoseData> odom_data_buff_;

  ImageData cur_img_0_;
  ImageData cur_img_1_;
  ImageData cur_img_2_;
  ImageData cur_img_3_;
  ImageData cur_img_4_;
  ImageData cur_img_5_;
  IMUData cur_imu_;
  PoseData cur_odom_;

  // extrinsic matrix
  Eigen::Matrix4f camera0_to_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f camera1_to_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f camera2_to_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f camera3_to_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f camera4_to_base_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f camera5_to_base_ = Eigen::Matrix4f::Identity();
};

} // namespace avp_mapping

#endif // _DATA_PRETREAT_FLOW_H_
