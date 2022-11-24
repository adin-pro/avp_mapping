/*
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:25:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:37:41
 */
#ifndef _REGISTRATION_INTERFACE_H_
#define _REGISTRATION_INTERFACE_H_

#include "Eigen/Dense"
#include "yaml-cpp/yaml.h"

#include "sensor_data/cloud_data.hpp"

namespace avp_mapping {
class RegistrationInterface {
public:
  virtual ~RegistrationInterface() = default;
  virtual bool setInputTarget(const CloudData::CLOUD_PTR &input_target) = 0;
  virtual bool scanMatch(const CloudData::CLOUD_PTR &input_source,
                         const Eigen::Matrix4d &predict_pose,
                         CloudData::CLOUD_PTR &result_cloud_ptr,
                         Eigen::Matrix4d &result_pose) = 0;
  virtual double getFitnessScore() = 0;
};
} // namespace avp_mapping

#endif // _REGISTRATION_INTERFACE_H_