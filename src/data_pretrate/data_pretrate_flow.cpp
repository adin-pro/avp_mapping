/*
 * @Author: ding.yin
 * @Date: 2022-10-15 19:56:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-06 16:25:12
 */

#include "data_pretreat/data_pretreat_flow.hpp"
#include <algorithm>
#include <chrono>

#include "glog/logging.h"
#include "models/cloud_filter/voxel_filter.hpp"

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
  // baselink_to_cameraX
  base_to_camera0_ptr_ =
      std::make_shared<TFListener>(nh, "/camera0_link", "/base_link");
  base_to_camera1_ptr_ =
      std::make_shared<TFListener>(nh, "/camera1_link", "/base_link");
  base_to_camera2_ptr_ =
      std::make_shared<TFListener>(nh, "/camera2_link", "/base_link");
  base_to_camera3_ptr_ =
      std::make_shared<TFListener>(nh, "/camera3_link", "/base_link");
  base_to_camera4_ptr_ =
      std::make_shared<TFListener>(nh, "/camera4_link", "/base_link");
  base_to_camera5_ptr_ =
      std::make_shared<TFListener>(nh, "/camera5_link", "/base_link");

  // publisher
  img_pub_ptr_ =
      std::make_shared<ImagePublisher>(nh, "/bev/image", "/map", 100);
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, "/bev/rgb_cloud", "/base_link", 100);

  camera_ = CameraModel(337.2084410968044, 337.2084410968044, 320.5, 240.5);

  filter_ptr_ = std::make_shared<VoxelFilter>(0.1, 0.1, 0.1);

  bev_cloud_ptr_ = CloudData::CLOUD_PTR(new CloudData::CLOUD());
  filtered_bev_cloud_ptr_ = CloudData::CLOUD_PTR(new CloudData::CLOUD());
}

bool DataPretreatFlow::run() {
  if (!readData())
    return false;

  if (!initCalibration())
    return false;

  // while (hasData()) {
  //   if (!validData())
  //     continue;
  //   transformData();
  //   publishData();
  // }

  if (hasData()) {
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
  // odom_sub_ptr_->parseData(unsynced_odom_data_buff);

  LOG(INFO) << "Image0 buffer size: " << img_data_buff_0_.size();
  LOG(INFO) << "Image1 buffer size: " << img_data_buff_1_.size();
  LOG(INFO) << "Image2 buffer size: " << img_data_buff_2_.size();
  LOG(INFO) << "Image3 buffer size: " << img_data_buff_3_.size();
  LOG(INFO) << "Image4 buffer size: " << img_data_buff_4_.size();
  LOG(INFO) << "Image5 buffer size: " << img_data_buff_5_.size();
  // LOG(INFO) << "Unsynced Odom buffer size: " << unsynced_odom_data_buff.size();
  // LOG(INFO) << "Synced Odom buffer size: " << odom_data_buff_.size();

  if (img_data_buff_0_.size() == 0 || img_data_buff_1_.size() == 0 ||
      img_data_buff_2_.size() == 0 || img_data_buff_3_.size() == 0 ||
      img_data_buff_4_.size() == 0 || img_data_buff_5_.size() == 0) {
    LOG(INFO) << "do not has enough image data";
    return false;
  }

  img_sync_time_ = img_data_buff_0_.front().time;

  LOG(INFO) << "image 0 " << img_data_buff_0_.front().time; 
  LOG(INFO) << "image 1 " << img_data_buff_1_.front().time; 
  LOG(INFO) << "image 2 " << img_data_buff_2_.front().time; 
  LOG(INFO) << "image 3 " << img_data_buff_3_.front().time; 
  LOG(INFO) << "image 4 " << img_data_buff_4_.front().time; 
  LOG(INFO) << "image 5 " << img_data_buff_5_.front().time; 

  img_sync_time_ = std::max(img_sync_time_, img_data_buff_1_.front().time);
  img_sync_time_ = std::max(img_sync_time_, img_data_buff_2_.front().time);
  img_sync_time_ = std::max(img_sync_time_, img_data_buff_3_.front().time);
  img_sync_time_ = std::max(img_sync_time_, img_data_buff_4_.front().time);
  img_sync_time_ = std::max(img_sync_time_, img_data_buff_5_.front().time);

  bool sync_0 = ImageData::syncData(img_data_buff_0_, img_sync_time_);
  bool sync_1 = ImageData::syncData(img_data_buff_1_, img_sync_time_);
  bool sync_2 = ImageData::syncData(img_data_buff_2_, img_sync_time_);
  bool sync_3 = ImageData::syncData(img_data_buff_3_, img_sync_time_);
  bool sync_4 = ImageData::syncData(img_data_buff_4_, img_sync_time_);
  bool sync_5 = ImageData::syncData(img_data_buff_5_, img_sync_time_);

  // bool sync_odom = PoseData::syncData(unsynced_odom_data_buff, odom_data_buff_,
  //                                     img_sync_time_);

  if (sync_0 && sync_1 && sync_2 && sync_3 && sync_4 && sync_5) {
    LOG(INFO) << "Sync time " << img_sync_time_;
    // if (!sync_odom) {
    //   LOG(INFO) << "Odom Sync failed";
    //   LOG(INFO) << "Odom time " << unsynced_odom_data_buff.front().time;
    //   pop_image_data();
    //   return false;
    // }
    return true;
  } else {
    LOG(INFO) << "Image Sync failed";
    return false;
  }
  return true;
}

bool DataPretreatFlow::initCalibration() {
  static bool calib_received = false;
  if (!calib_received) {
    if (base_to_camera0_ptr_->lookUpData(base_to_camera0_) &&
        base_to_camera1_ptr_->lookUpData(base_to_camera1_) &&
        base_to_camera2_ptr_->lookUpData(base_to_camera2_) &&
        base_to_camera3_ptr_->lookUpData(base_to_camera3_) &&
        base_to_camera4_ptr_->lookUpData(base_to_camera4_) &&
        base_to_camera5_ptr_->lookUpData(base_to_camera5_)) {
      calib_received = true;
    }
  }

  return calib_received;
}

bool DataPretreatFlow::hasData() {
  if (img_data_buff_0_.size() == 0 || img_data_buff_1_.size() == 0 ||
      img_data_buff_2_.size() == 0 || img_data_buff_3_.size() == 0 ||
      img_data_buff_4_.size() == 0 || img_data_buff_5_.size() == 0) {
    LOG(INFO) << "do not has image data";
    return false;
  }

  // if (unsynced_odom_data_buff.size() == 0) {
  //   LOG(INFO) << "do not has odom data";
  //   return false;
  // }

  return true;
}

bool DataPretreatFlow::validData() { return true; }

bool DataPretreatFlow::pop_image_data() {
  if (img_data_buff_0_.size() == 0 || img_data_buff_1_.size() == 0 ||
      img_data_buff_2_.size() == 0 || img_data_buff_3_.size() == 0 ||
      img_data_buff_4_.size() == 0 || img_data_buff_5_.size() == 0) {
    LOG(INFO) << "do not has image data to pop";
    return false;
  }
  img_data_buff_0_.pop_front();
  img_data_buff_1_.pop_front();
  img_data_buff_2_.pop_front();
  img_data_buff_3_.pop_front();
  img_data_buff_4_.pop_front();
  img_data_buff_5_.pop_front();
  return true;
}

bool DataPretreatFlow::publishData() {
  auto clock_start = std::chrono::system_clock::now();

  LOG(INFO) << "publish image";
  cv::Mat bev_image(cv::Size(400, 400), CV_8UC3, cv::Scalar(0, 0, 0));
  float scale = 0.05;
  camera_.img2BevImage(img_data_buff_0_.front().image, bev_image,
                       base_to_camera0_, scale);
  camera_.img2BevImage(img_data_buff_1_.front().image, bev_image,
                       base_to_camera1_, scale);
  camera_.img2BevImage(img_data_buff_2_.front().image, bev_image,
                       base_to_camera2_, scale);
  camera_.img2BevImage(img_data_buff_3_.front().image, bev_image,
                       base_to_camera3_, scale);
  camera_.img2BevImage(img_data_buff_4_.front().image, bev_image,
                       base_to_camera4_, scale);
  camera_.img2BevImage(img_data_buff_5_.front().image, bev_image,
                       base_to_camera5_, scale);

  img_pub_ptr_->publish(bev_image, img_sync_time_);

  auto clock_img_finish = std::chrono::system_clock::now();
  // point 2 cloud
  bev_cloud_ptr_->clear();
  camera_.img2BevCloud(img_data_buff_0_.front().image, bev_cloud_ptr_,
                       base_to_camera0_);
  camera_.img2BevCloud(img_data_buff_1_.front().image, bev_cloud_ptr_,
                       base_to_camera1_);
  camera_.img2BevCloud(img_data_buff_2_.front().image, bev_cloud_ptr_,
                       base_to_camera2_);
  camera_.img2BevCloud(img_data_buff_3_.front().image, bev_cloud_ptr_,
                       base_to_camera3_);
  camera_.img2BevCloud(img_data_buff_4_.front().image, bev_cloud_ptr_,
                       base_to_camera4_);
  camera_.img2BevCloud(img_data_buff_5_.front().image, bev_cloud_ptr_,
                       base_to_camera5_);
  auto cloud_transform_finish = std::chrono::system_clock::now();
  // filter
  filtered_bev_cloud_ptr_->clear();
  LOG(INFO) << "bev ipm cloud: " << bev_cloud_ptr_->points.size();
  filter_ptr_->filter(bev_cloud_ptr_, filtered_bev_cloud_ptr_);
  LOG(INFO) << "filtered ipm cloud: " << filtered_bev_cloud_ptr_->points.size();
  auto filter_finish = std::chrono::system_clock::now();
  cloud_pub_ptr_->publish(filtered_bev_cloud_ptr_);
  auto publish_cloud_finish = std::chrono::system_clock::now();
  pop_image_data();
  auto pop_finish = std::chrono::system_clock::now();

  LOG(INFO) << "publish image: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          clock_img_finish - clock_start)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;
  LOG(INFO) << "cloud transform: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          cloud_transform_finish - clock_img_finish)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;
  LOG(INFO) << "cloud filter: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          filter_finish - cloud_transform_finish)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;
  LOG(INFO) << "publish_cloud_finish: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          publish_cloud_finish - filter_finish)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;
  LOG(INFO) << "pop_finish: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          pop_finish - publish_cloud_finish)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;

  return true;
}

bool DataPretreatFlow::transformData() { return true; }

} // namespace avp_mapping
