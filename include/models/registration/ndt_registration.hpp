/*
 * @Author: ding.yin
 * @Date: 2022-10-15 10:54:38
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-15 16:39:16
 */
#ifndef _NDT_REGISTRATION_H_
#define _NDT_REGISTRATION_H_

#include "pcl/registration/ndt.h"

#include "models/registration/registration_interface.hpp"

namespace avp_mapping {

class NDTRegistration : public RegistrationInterface {

public:
  NDTRegistration(const YAML::Node &node);
  NDTRegistration(float res, float step_size, float trans_eps, int max_iter);

  bool setInputTarget(const CloudData::CLOUD_PTR &taget_ptr) override;
  bool scanMatch(const CloudData::CLOUD_PTR &input_source,
                 const Eigen::Matrix4f &predict_pose,
                 CloudData::CLOUD_PTR &result_cloud_ptr,
                 Eigen::Matrix4f &result_pose) override;

  float getFitnessScore() override;

private:
  bool setRegistrationParam(float res, float step_size, float trans_eps,
                            int max_iter);

private:
  pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr
      ndt_ptr_;
};

} // namespace avp_mapping

#endif // _NDT_REGISTRATION_H_