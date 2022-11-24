/*
 * @Author: ding.yin
 * @Date: 2022-11-05 16:14:01
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-06 14:07:19
 */

#include "mapping/front_end/front_end.hpp"

#include "glog/logging.h"
#include "models/cloud_filter/voxel_filter.hpp"
#include "models/registration/ndt_registration.hpp"

namespace avp_mapping {

FrontEnd::FrontEnd(double key_frame_dist, int local_frame_num)
    : local_map_ptr_(new CloudData::CLOUD()),
      key_frame_distance_(key_frame_dist),
      local_frame_num_(local_frame_num) {
  LOG(INFO) << "----------- Front End Initalization ------------";
  LOG(INFO) << "Registration Method: ";
  initRegistration(registration_ptr_, "NDT");
  LOG(INFO) << "Local Map Filter: ";
  initFilter(local_map_filter_ptr_, "voxel_filter", 0.1);
  LOG(INFO) << "Key Frame Filter: ";
  initFilter(frame_filter_ptr_, "voxel_filter", 0.05);
}

bool FrontEnd::initRegistration(
    std::shared_ptr<RegistrationInterface> &registration_ptr,
    const std::string &type) {
  if (type == "NDT") {
    // NDT params
    double res = 1.0;
    double step_size = 0.1;
    double trans_eps = 0.01;
    int max_iter = 30;
    registration_ptr =
        std::make_shared<NDTRegistration>(res, step_size, trans_eps, max_iter);
  } else {

    LOG(ERROR) << "Not Implemented: " << type;
    return false;
  }
  return true;
}

bool FrontEnd::initFilter(std::shared_ptr<CloudFilterInterface> &filter_ptr,
                          const std::string &type, double leaf_size) {
  if (type == "voxel_filter") {
    filter_ptr = std::make_shared<VoxelFilter>(leaf_size, leaf_size, leaf_size);
  } else if (type == "no_filter") {
    LOG(ERROR) << "Not Implemented " << type;
  } else {
    LOG(ERROR) << "Not Implemented " << type;
  }
  return true;
}

bool FrontEnd::update(const CloudData &cloud_data,
                      Eigen::Matrix4d &result_pose) {
  current_frame_.cloud_data.time = cloud_data.time;
  // remove Nan
  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*cloud_data.cloud_ptr,
                               *current_frame_.cloud_data.cloud_ptr, indices);

  CloudData::CLOUD_PTR filtered_cloud_ptr(new CloudData::CLOUD());
  frame_filter_ptr_->filter(current_frame_.cloud_data.cloud_ptr,
                            filtered_cloud_ptr);

  static Eigen::Matrix4d step_pose = Eigen::Matrix4d::Identity();
  static Eigen::Matrix4d last_pose = init_pose_;
  static Eigen::Matrix4d predict_pose = init_pose_;
  static Eigen::Matrix4d last_key_frame_pose = init_pose_;

  // first key frame in local map deques
  if (local_map_frames_.size() == 0) {
    current_frame_.pose = init_pose_;
    udpateWithNewKeyFrame(current_frame_);
    result_pose = init_pose_;
    return true;
  }

  CloudData::CLOUD_PTR result_cloud_ptr(new CloudData::CLOUD());
  registration_ptr_->scanMatch(filtered_cloud_ptr, predict_pose,
                               result_cloud_ptr, current_frame_.pose);
  result_pose = current_frame_.pose;
  step_pose = last_pose.inverse() * result_pose;
  predict_pose = result_pose * step_pose;
  last_pose = current_frame_.pose;

  if (fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
          fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) +
          fabs(last_key_frame_pose(0, 3) - current_frame_.pose(0, 3)) >
      key_frame_distance_) {
    udpateWithNewKeyFrame(current_frame_);
    last_key_frame_pose = current_frame_.pose;
  }

  return true;
}

bool FrontEnd::udpateWithNewKeyFrame(const Frame &new_key_frame) {
  // shallow copy
  Frame key_frame = new_key_frame;
  // deep copy
  key_frame.cloud_data.cloud_ptr.reset(
      new CloudData::CLOUD(*new_key_frame.cloud_data.cloud_ptr));

  local_map_frames_.push_back(key_frame);
  while (local_map_frames_.size() > static_cast<size_t>(local_frame_num_)) {
    local_map_frames_.pop_front();
  }

  CloudData::CLOUD_PTR transformed_cloud_ptr(new CloudData::CLOUD());
  // new local map point cloud
  local_map_ptr_.reset(new CloudData::CLOUD());
  for (size_t i = 0; i < local_map_frames_.size(); ++i) {
    pcl::transformPointCloud(*local_map_frames_[i].cloud_data.cloud_ptr,
                             *transformed_cloud_ptr, local_map_frames_[i].pose);
    *local_map_ptr_ += *transformed_cloud_ptr;
  }

  // update ndt target point cloud
  if (local_map_frames_.size() < 10) {
    registration_ptr_->setInputTarget(local_map_ptr_);
  } else {
    CloudData::CLOUD_PTR filtered_local_map_ptr(new CloudData::CLOUD());
    local_map_filter_ptr_->filter(local_map_ptr_, filtered_local_map_ptr);
    registration_ptr_->setInputTarget(filtered_local_map_ptr);
  }

  return true;
}

bool FrontEnd::setInitPose(const Eigen::Matrix4d init_pose) {
  init_pose_ = init_pose;
  return true;
}

} // namespace avp_mapping