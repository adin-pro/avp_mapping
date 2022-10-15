/*
 * @Author: ding.yin
 * @Date: 2022-10-15 16:34:19
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-15 19:43:50
 */

#include "models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace avp_mapping {

VoxelFilter::VoxelFilter(const YAML::Node &node) {
  float leaf_size_x = node["leaf_size"][0].as<float>();
  float leaf_size_y = node["leaf_size"][1].as<float>();
  float leaf_size_z = node["leaf_size"][2].as<float>();
  setFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(float leaf_size_x, float leaf_size_y,
                         float leaf_size_z) {
  setFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::setFilterParam(float leaf_size_x, float leaf_size_y,
                                 float leaf_size_z) {
  voxel_filter_.setLeafSize(leaf_size_x, leaf_size_y, leaf_size_z);
  LOG(INFO) << "VoxelFilter";
  LOG(INFO) << "leafsize [x,y,z]: " << leaf_size_x << " " << leaf_size_y << " "
            << leaf_size_z;
  return true;
}

bool VoxelFilter::filter(const CloudData::CLOUD_PTR &input_cloud_ptr,
                         CloudData::CLOUD_PTR &filtered_cloud_ptr) {
  voxel_filter_.setInputCloud(input_cloud_ptr);
  voxel_filter_.filter(*filtered_cloud_ptr);
  return true;
}

} // namespace avp_mapping
