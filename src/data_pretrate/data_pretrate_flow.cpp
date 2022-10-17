/*
 * @Author: ding.yin
 * @Date: 2022-10-15 19:56:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:30:46
 */

#include <algorithm>

#include "glog/logging.h"

#include "data_pretreat/data_pretreat_flow.hpp"

namespace avp_mapping {

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh) {
  // subscriber
  img_sub_ptr_0_ =
      std::make_shared<ImageSubscriber>(nh, "/camera0/color/image_raw", 100);
  img_sub_ptr_1_ =
      std::make_shared<ImageSubscriber>(nh, "/camera1/color/image_raw", 100);
  img_sub_ptr_2_ =
      std::make_shared<ImageSubscriber>(nh, "/camera2/color/image_raw", 100);
  img_sub_ptr_3_ =
      std::make_shared<ImageSubscriber>(nh, "/camera3/color/image_raw", 100);
  img_sub_ptr_4_ =
      std::make_shared<ImageSubscriber>(nh, "/camera4/color/image_raw", 100);
  img_sub_ptr_5_ =
      std::make_shared<ImageSubscriber>(nh, "/camera5/color/image_raw", 100);
  odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(nh, "/odom", 100);

  // TF listener
  camera0_to_base_ptr_ =
      std::make_shared<TFListener>(nh, "/base_link", "/camera0_link");

  // publisher
  img_pub_ptr_ =
      std::make_shared<ImagePublisher>(nh, "/bev/image", "/map", 100);
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/bev/rgb_cloud", "/velo_link", 100);
}

bool DataPretreatFlow::run() {
  if (!readData())
    return false;

  if (!initCalibration())
    return false;

  while (hasData()) {
    // if (!validData())
    //   continue;
    transformData();
    publishData();
  }
  return true;
}

bool DataPretreatFlow::readData() {
  img_sub_ptr_0_->parseData(img_data_buff_0_);
  img_sub_ptr_1_->parseData(img_data_buff_1_);
  img_sub_ptr_2_->parseData(img_data_buff_2_);
  img_sub_ptr_3_->parseData(img_data_buff_3_);
  img_sub_ptr_4_->parseData(img_data_buff_4_);
  img_sub_ptr_5_->parseData(img_data_buff_5_);

  odom_sub_ptr_->parseData(odom_data_buff_);

  if (img_data_buff_0_.size() == 0) {
    // LOG(ERROR) << "Empty Image buffer";
    return false;
  }

  double img_sync_time = img_data_buff_0_.front().time;
  img_sync_time = std::max(img_sync_time, img_data_buff_1_.front().time);
  img_sync_time = std::max(img_sync_time, img_data_buff_2_.front().time);
  img_sync_time = std::max(img_sync_time, img_data_buff_3_.front().time);
  img_sync_time = std::max(img_sync_time, img_data_buff_4_.front().time);
  img_sync_time = std::max(img_sync_time, img_data_buff_5_.front().time);

  // LOG(INFO) << "img 0 " << img_data_buff_0_.front().time << " "
  //           << img_data_buff_0_.size();
  // LOG(INFO) << "img 1 " << img_data_buff_1_.front().time << " "
  //           << img_data_buff_1_.size();
  // LOG(INFO) << "img 2 " << img_data_buff_2_.front().time << " "
  //           << img_data_buff_2_.size();
  // LOG(INFO) << "img 3 " << img_data_buff_3_.front().time << " "
  //           << img_data_buff_3_.size();
  // LOG(INFO) << "img 4 " << img_data_buff_4_.front().time << " "
  //           << img_data_buff_4_.size();
  // LOG(INFO) << "img 5 " << img_data_buff_5_.front().time << " "
  //           << img_data_buff_5_.size();

  bool sync_0 = ImageData::syncData(img_data_buff_0_, img_sync_time);
  bool sync_1 = ImageData::syncData(img_data_buff_1_, img_sync_time);
  bool sync_2 = ImageData::syncData(img_data_buff_2_, img_sync_time);
  bool sync_3 = ImageData::syncData(img_data_buff_3_, img_sync_time);
  bool sync_4 = ImageData::syncData(img_data_buff_4_, img_sync_time);
  bool sync_5 = ImageData::syncData(img_data_buff_5_, img_sync_time);

  if (sync_0 && sync_1 && sync_2 && sync_3 && sync_4 && sync_5) {
    LOG(INFO) << "Sync time " << img_sync_time;
    return true;
  } else {
    LOG(ERROR) << "Sync failed";
    return false;
  }
}

bool DataPretreatFlow::initCalibration() {
  static bool calib_received = false;
  if (!calib_received) {
    if (camera0_to_base_ptr_->lookUpData(camera0_to_base_)) {
      calib_received = true;
    }
  }

  return calib_received;
}

bool DataPretreatFlow::hasData() {
  if (img_data_buff_0_.size() == 0 || img_data_buff_1_.size() == 0 ||
      img_data_buff_2_.size() == 0 || img_data_buff_3_.size() == 0 ||
      img_data_buff_4_.size() == 0 || img_data_buff_5_.size() == 0)
    return false;

  if (odom_data_buff_.size() == 0)
    return false;

  return true;
}

bool DataPretreatFlow::validData() { return true; }

bool DataPretreatFlow::publishData() {
  LOG(INFO) << camera0_to_base_;
  ImageData img_data = img_data_buff_0_.front();
  img_data_buff_0_.pop_front();
  img_data_buff_1_.pop_front();
  img_data_buff_2_.pop_front();
  img_data_buff_3_.pop_front();
  img_data_buff_4_.pop_front();
  img_data_buff_5_.pop_front();
  img_pub_ptr_->publish(img_data.image.clone(), img_data.time);
  // LOG(INFO) << "Image buffer size: " << img_data_buff_0_.size();
  return true;
}

bool DataPretreatFlow::transformData() { return true; }

} // namespace avp_mapping
