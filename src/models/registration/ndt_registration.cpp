/*
 * @Author: ding.yin
 * @Date: 2022-10-15 15:54:50
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-15 16:43:43
 */

#include "models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace avp_mapping {

NDTRegistration::NDTRegistration(const ::YAML::Node &node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT,
                                                     CloudData::POINT>()) {

  float res = node["res"].as<float>();
  float step_size = node["step_size"].as<float>();
  float trans_eps = node["trans_eps"].as<float>();
  int max_iter = node["max_iter"].as<int>();

  setRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(float res, float step_size, float trans_eps,
                                 int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT,
                                                     CloudData::POINT>()) {
  setRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::setRegistrationParam(float res, float step_size,
                                           float trans_eps, int max_iter) {
  ndt_ptr_->setResolution(res);
  ndt_ptr_->setStepSize(step_size);
  ndt_ptr_->setTransformationEpsilon(trans_eps);
  ndt_ptr_->setMaximumIterations(max_iter);
  LOG(INFO) << "NDT Registration";
  LOG(INFO) << "resolution: " << res;
  LOG(INFO) << "step_size: " << step_size;
  LOG(INFO) << "transformation epsilon: " << trans_eps;
  LOG(INFO) << "maximum iteration: " << max_iter;
  return true;
}

bool NDTRegistration::setInputTarget(const CloudData::CLOUD_PTR &input_target) {
  ndt_ptr_->setInputTarget(input_target);
  return true;
}

bool NDTRegistration::scanMatch(const CloudData::CLOUD_PTR &input_source,
                                const Eigen::Matrix4f &predict_pose,
                                CloudData::CLOUD_PTR &filtered_cloud_ptr,
                                Eigen::Matrix4f &result_pose) {
  ndt_ptr_->setInputSource(input_source);
  ndt_ptr_->align(*filtered_cloud_ptr, predict_pose);
  result_pose = ndt_ptr_->getFinalTransformation();
  return true;
}

float NDTRegistration::getFitnessScore() { return ndt_ptr_->getFitnessScore(); }

} // namespace avp_mapping
