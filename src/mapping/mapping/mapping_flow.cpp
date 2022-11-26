/*
 * @Author: ding.yin
 * @Date: 2022-11-09 20:37:37
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-25 17:51:53
 */

#include "mapping/mapping/mapping_flow.hpp"

#include "glog/logging.h"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "yaml-cpp/yaml.h"

namespace avp_mapping {
MappingFlow::MappingFlow(ros::NodeHandle &nh, const std::string work_dir) {
  YAML::Node node = YAML::LoadFile(work_dir + "/config/mapping/mapping.yaml");

  kf_cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(
      nh, node["kf_cloud_sub_topic"].as<std::string>(), 1000);

  back_end_kfs_sub_ptr_ = std::make_shared<KeyFramesSubscriber>(
      nh, node["back_end_kfs_sub_topic"].as<std::string>(), 1000);

  front_end_kf_sub_ptr_ = std::make_shared<KeyFrameSubscriber>(
      nh, node["front_end_kf_sub_topic"].as<std::string>(), 1000);

  optimized_cloud_map_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, node["optimized_cloud_map_pub_topic"].as<std::string>(),
      node["frame_id"].as<std::string>(), 1000);

  original_cloud_map_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, node["original_cloud_map_pub_topic"].as<std::string>(),
      node["frame_id"].as<std::string>(), 1000);

  save_map_ = node["save_map"].as<bool>();
  save_dir_ = node["save_dir"].as<std::string>();
}

bool MappingFlow::run() {
  if (!readData())
    return false;
  if (hasData() && validData()) {
    transformData();
    publishData();
  }
  return true;
}

bool MappingFlow::readData() {
  kf_cloud_sub_ptr_->parseData(kf_cloud_data_buff_);
  back_end_kfs_sub_ptr_->parseData(back_end_kfs_data_buff_);
  front_end_kf_sub_ptr_->parseData(front_end_kf_data_buff_);
  return true;
}

bool MappingFlow::hasData() {
  if (kf_cloud_data_buff_.size() > 0 && back_end_kfs_data_buff_.size() > 0 &&
      front_end_kf_data_buff_.size() > 0)
    return true;
  return false;
}

bool MappingFlow::validData() {
  if (kf_cloud_data_buff_.size() == back_end_kfs_data_buff_.size()) {
    LOG(INFO) << "Number of KeyFrames: " << back_end_kfs_data_buff_.size();
    return true;
  }
  return false;
}

bool MappingFlow::transformData() {
  optimized_cloud_map_.cloud_ptr->clear();
  original_cloud_map_.cloud_ptr->clear();
  CloudData::CLOUD_PTR transformed_sub_cloud(new CloudData::CLOUD());
  for (size_t si = 0; si < kf_cloud_data_buff_.size(); si++) {
    transformed_sub_cloud->clear();
    pcl::transformPointCloud(*kf_cloud_data_buff_[si].cloud_ptr,
                             *transformed_sub_cloud,
                             back_end_kfs_data_buff_[si].pose);
    *optimized_cloud_map_.cloud_ptr += *transformed_sub_cloud;
    pcl::transformPointCloud(*kf_cloud_data_buff_[si].cloud_ptr,
                             *transformed_sub_cloud,
                             front_end_kf_data_buff_[si].pose);
    *original_cloud_map_.cloud_ptr += *transformed_sub_cloud;
  }
  back_end_kfs_data_buff_.clear();
}

bool MappingFlow::publishData() {
  optimized_cloud_map_pub_ptr_->publish(optimized_cloud_map_.cloud_ptr);
  original_cloud_map_pub_ptr_->publish(original_cloud_map_.cloud_ptr);
  if (save_map_) {
    pcl::io::savePCDFileBinary(save_dir_ + "optimized_map.pcd",
                               *optimized_cloud_map_.cloud_ptr);
    pcl::io::savePCDFileBinary(save_dir_ + "original_map.pcd",
                               *original_cloud_map_.cloud_ptr);
  }
  return true;
}

} // namespace avp_mapping