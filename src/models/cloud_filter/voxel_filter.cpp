/*
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:53:20
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:44:02
 */

#include "models/cloud_filter/voxel_filter.hpp"

#include "glog/logging.h"

namespace avp_mapping {

VoxelFilter::VoxelFilter(const YAML::Node &node) {
  double leaf_size_x = node["leaf_size"][0].as<double>();
  double leaf_size_y = node["leaf_size"][1].as<double>();
  double leaf_size_z = node["leaf_size"][2].as<double>();
  setFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

VoxelFilter::VoxelFilter(double leaf_size_x, double leaf_size_y,
                         double leaf_size_z) {
  setFilterParam(leaf_size_x, leaf_size_y, leaf_size_z);
}

bool VoxelFilter::setFilterParam(double leaf_size_x, double leaf_size_y,
                                 double leaf_size_z) {
  voxel_filter_.setLeafSize(static_cast<float>(leaf_size_x),
                            static_cast<float>(leaf_size_y),
                            static_cast<float>(leaf_size_z));
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
