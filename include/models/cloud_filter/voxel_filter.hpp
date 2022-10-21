/*
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:37:49
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:37:02
 */
#ifndef _VOXEL_FILTER_H_
#define _VOXEL_FILTER_H_

#include "pcl/filters/voxel_grid.h"

#include "models/cloud_filter/cloud_filter_interface.hpp"

namespace avp_mapping {

class VoxelFilter : public CloudFilterInterface {
public:
  VoxelFilter(const YAML::Node &node);
  VoxelFilter(float leaf_size_x, float leaf_size_y, float leaf_size_z);

  bool filter(const CloudData::CLOUD_PTR &input_cloud_ptr,
              CloudData::CLOUD_PTR &filtered_cloud_ptr) override;

private:
  bool setFilterParam(float leaf_size_x, float leaf_size_y, float leaf_size_z);

private:
  pcl::VoxelGrid<CloudData::POINT> voxel_filter_;
};

} // namespace avp_mapping

#endif // _VOXEL_FILTER_H_