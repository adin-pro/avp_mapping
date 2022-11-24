/*
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:57
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:37:23
 */
#ifndef _NDT_REGISTRATION_H_
#define _NDT_REGISTRATION_H_

#include "pcl/registration/ndt.h"

#include "models/registration/registration_interface.hpp"

namespace avp_mapping {

class NDTRegistration : public RegistrationInterface {

public:
  NDTRegistration(const YAML::Node &node);
  NDTRegistration(double res, double step_size, double trans_eps, int max_iter);

  bool setInputTarget(const CloudData::CLOUD_PTR &taget_ptr) override;
  bool scanMatch(const CloudData::CLOUD_PTR &input_source,
                 const Eigen::Matrix4d &predict_pose,
                 CloudData::CLOUD_PTR &result_cloud_ptr,
                 Eigen::Matrix4d &result_pose) override;

  double getFitnessScore() override;

private:
  bool setRegistrationParam(double res, double step_size, double trans_eps,
                            int max_iter);

private:
  pcl::NormalDistributionsTransform<CloudData::POINT, CloudData::POINT>::Ptr
      ndt_ptr_;
};

} // namespace avp_mapping

#endif // _NDT_REGISTRATION_H_