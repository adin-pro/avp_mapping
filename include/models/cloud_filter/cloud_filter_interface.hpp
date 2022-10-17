/*
 * @Author: Ren Qian
 * @Date: 2020-02-09 19:29:50
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:36:40
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