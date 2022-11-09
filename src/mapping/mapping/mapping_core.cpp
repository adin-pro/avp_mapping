/*
 * @Author: ding.yin
 * @Date: 2022-11-09 21:09:21
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 21:10:58
 */

#include "mapping/mapping/mapping_core.hpp"

namespace avp_mapping {

MappingCore::MappingCore() {}

bool MappingCore::vectorization(CloudData::CLOUD_PTR &map_cloud_ptr) {
  return true;
}

bool MappingCore::quatTreeVoxelization(CloudData::CLOUD_PTR &map_cloud_ptr) {
  return true;
}

} // namespace avp_mapping