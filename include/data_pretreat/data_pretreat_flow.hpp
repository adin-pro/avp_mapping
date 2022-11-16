/*
 * @Author: ding.yin
 * @Date: 2022-10-02 14:23:54
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-06 13:59:33
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
#include "publisher/odometry_publisher.hpp"

#include "models/camera/camera_model.hpp"
#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace avp_mapping {

class DataPretreatFlow {

public:
  DataPretreatFlow(ros::NodeHandle &nh, std::string work_dir);

  bool run();

private:
  bool readData();
  bool initCalibration();
  bool hasData();
  bool validData();
  bool publishData();
  bool transformData();
  bool pop_image_data();

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
  std::shared_ptr<TFListener> base_to_camera0_ptr_;
  std::shared_ptr<TFListener> base_to_camera1_ptr_;
  std::shared_ptr<TFListener> base_to_camera2_ptr_;
  std::shared_ptr<TFListener> base_to_camera3_ptr_;
  std::shared_ptr<TFListener> base_to_camera4_ptr_;
  std::shared_ptr<TFListener> base_to_camera5_ptr_;

  // publisher
  std::shared_ptr<CloudPublisher> cloud_pub_ptr_;
  std::shared_ptr<ImagePublisher> img_pub_ptr_;
  std::shared_ptr<TFBroadCaster> tfbroad_pub_ptr_;
  std::shared_ptr<OdometryPublisher> odom_pub_ptr_;


  std::deque<ImageData> img_data_buff_0_;
  std::deque<ImageData> img_data_buff_1_;
  std::deque<ImageData> img_data_buff_2_;
  std::deque<ImageData> img_data_buff_3_;
  std::deque<ImageData> img_data_buff_4_;
  std::deque<ImageData> img_data_buff_5_;
  std::deque<IMUData> imu_data_buff_;
  std::deque<PoseData> odom_data_buff_;
  std::deque<PoseData> unsynced_odom_data_buff;

  ImageData cur_img_0_;
  ImageData cur_img_1_;
  ImageData cur_img_2_;
  ImageData cur_img_3_;
  ImageData cur_img_4_;
  ImageData cur_img_5_;
  IMUData cur_imu_;
  PoseData cur_odom_;

  // extrinsic matrix
  Eigen::Matrix4f base_to_camera0_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_camera1_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_camera2_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_camera3_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_camera4_ = Eigen::Matrix4f::Identity();
  Eigen::Matrix4f base_to_camera5_ = Eigen::Matrix4f::Identity();

  CameraModel camera_;

  double img_sync_time_ = 0.0;

  float scale_ = 0.05;
  int image_width_ = 280;
  int image_height_ = 280;

  std::shared_ptr<CloudFilterInterface> filter_ptr_;

  CloudData::CLOUD_PTR bev_cloud_ptr_;
  CloudData::CLOUD_PTR filtered_bev_cloud_ptr_;
};

} // namespace avp_mapping

#endif // _DATA_PRETREAT_FLOW_H_
