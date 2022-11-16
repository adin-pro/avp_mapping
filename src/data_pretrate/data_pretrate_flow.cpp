/*
 * @Author: ding.yin
 * @Date: 2022-10-15 19:56:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 15:40:48
 */

#include "data_pretreat/data_pretreat_flow.hpp"
#include <algorithm>
#include <chrono>

#include "glog/logging.h"
#include "models/cloud_filter/voxel_filter.hpp"
#include "yaml-cpp/yaml.h"

namespace avp_mapping {

DataPretreatFlow::DataPretreatFlow(ros::NodeHandle &nh, std::string work_dir) {
  YAML::Node node =
      YAML::LoadFile(work_dir + "/config/mapping/data_pretreat.yaml");
  std::string image_topic = node["image_topic"].as<std::string>();
  std::string image_pub_topic = node["image_pub_topic"].as<std::string>();
  std::string cloud_pub_topic = node["cloud_pub_topic"].as<std::string>();
  
  // bev_image
  scale_ = node["image_scale"].as<float>();
  image_height_ = node["image_height"].as<int>();
  image_width_ = node["image_width"].as<int>();

  // subscriber
  img_sub_ptr_0_ =
      std::make_shared<ImageSubscriber>(nh, "/camera0" + image_topic, 100);
  img_sub_ptr_1_ =
      std::make_shared<ImageSubscriber>(nh, "/camera1" + image_topic, 100);
  img_sub_ptr_2_ =
      std::make_shared<ImageSubscriber>(nh, "/camera2" + image_topic, 100);
  img_sub_ptr_3_ =
      std::make_shared<ImageSubscriber>(nh, "/camera3" + image_topic, 100);
  img_sub_ptr_4_ =
      std::make_shared<ImageSubscriber>(nh, "/camera4" + image_topic, 100);
  img_sub_ptr_5_ =
      std::make_shared<ImageSubscriber>(nh, "/camera5" + image_topic, 100);
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
      std::make_shared<ImagePublisher>(nh, image_pub_topic, "/map", 100);
  cloud_pub_ptr_ =
      std::make_shared<CloudPublisher>(nh, cloud_pub_topic, "/map", 100);

  camera_ = CameraModel(node["camera_model"]);

  filter_ptr_ = std::make_shared<VoxelFilter>(node["voxel_filter"]);

  bev_cloud_ptr_ = CloudData::CLOUD_PTR(new CloudData::CLOUD());
  filtered_bev_cloud_ptr_ = CloudData::CLOUD_PTR(new CloudData::CLOUD());
}

bool DataPretreatFlow::run() {
  if (!readData())
    return false;

  if (!initCalibration())
    return false;

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

  if (img_data_buff_0_.size() == 0 || img_data_buff_1_.size() == 0 ||
      img_data_buff_2_.size() == 0 || img_data_buff_3_.size() == 0 ||
      img_data_buff_4_.size() == 0 || img_data_buff_5_.size() == 0) {
    return false;
  }

  img_sync_time_ = img_data_buff_0_.front().time;

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

  if (sync_0 && sync_1 && sync_2 && sync_3 && sync_4 && sync_5) {
    LOG(INFO) << "Sync time " << img_sync_time_;
    return true;
  } else {
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
  cv::Mat bev_image(cv::Size(image_height_, image_width_), CV_8UC3, cv::Scalar(0, 0, 0));
  camera_.img2BevImage(img_data_buff_0_.front().image, bev_image,
                       base_to_camera0_, scale_);
  camera_.img2BevImage(img_data_buff_1_.front().image, bev_image,
                       base_to_camera1_, scale_);
  camera_.img2BevImage(img_data_buff_2_.front().image, bev_image,
                       base_to_camera2_, scale_);
  camera_.img2BevImage(img_data_buff_3_.front().image, bev_image,
                       base_to_camera3_, scale_);
  camera_.img2BevImage(img_data_buff_4_.front().image, bev_image,
                       base_to_camera4_, scale_);
  camera_.img2BevImage(img_data_buff_5_.front().image, bev_image,
                       base_to_camera5_, scale_);
  img_pub_ptr_->publish(bev_image, img_sync_time_);

  // point cloud
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
  // filter
  filtered_bev_cloud_ptr_->clear();
  LOG(INFO) << "bev ipm cloud: " << bev_cloud_ptr_->points.size();
  filter_ptr_->filter(bev_cloud_ptr_, filtered_bev_cloud_ptr_);
  LOG(INFO) << "filtered ipm cloud: " << filtered_bev_cloud_ptr_->points.size();
  cloud_pub_ptr_->publish(bev_cloud_ptr_, img_sync_time_);
  pop_image_data();

  return true;
}

bool DataPretreatFlow::transformData() { return true; }

} // namespace avp_mapping
