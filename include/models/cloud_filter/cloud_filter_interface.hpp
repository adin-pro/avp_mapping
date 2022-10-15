/*
 * @Author: ding.yin
 * @Date: 2022-10-15 10:46:22
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-15 10:51:15
 */
#ifndef _CLOUD_FILTER_INTERFACE_H_
#define _CLOUD_FILTER_INTERFACE_H_

#include "sensor_data/cloud_data.hpp"
#include "yaml-cpp/yaml.h"

namespace avp_mapping {
class CloudFilterInterface {

public:
  virtual ~CloudFilterInterface() = default;
  virtual bool filter(const CloudData::CLOUD_PTR &input_cloud_ptr,
                      CloudData::CLOUD_PTR &filtered_cloud_ptr) = 0;
};

} // namespace avp_mapping

#endif // _CLOUD_FILTER_INTERFACE_H_