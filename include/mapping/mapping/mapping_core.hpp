/*
 * @Author: ding.yin
 * @Date: 2022-11-09 21:03:29
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-09 21:10:21
 */

#ifndef _MAPPING_CORE_H_
#define _MAPPING_CORE_H_

#include "sensor_data/cloud_data.hpp"

namespace avp_mapping {
class MappingCore {
public:
  MappingCore();

  // TODO
  bool vectorization(CloudData::CLOUD_PTR &map_cloud_ptr);

  bool quatTreeVoxelization(CloudData::CLOUD_PTR &map_cloud_ptr);
};

} // namespace avp_mapping

#endif // _MAPPING_CORE_H_