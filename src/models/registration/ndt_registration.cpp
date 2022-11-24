/*
 * @Author: Ren Qian
 * @Date: 2020-02-08 21:46:45
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:44:23
 */

#include "models/registration/ndt_registration.hpp"

#include "glog/logging.h"

namespace avp_mapping {

NDTRegistration::NDTRegistration(const ::YAML::Node &node)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT,
                                                     CloudData::POINT>()) {

  double res = node["res"].as<double>();
  double step_size = node["step_size"].as<double>();
  double trans_eps = node["trans_eps"].as<double>();
  int max_iter = node["max_iter"].as<int>();

  setRegistrationParam(res, step_size, trans_eps, max_iter);
}

NDTRegistration::NDTRegistration(double res, double step_size, double trans_eps,
                                 int max_iter)
    : ndt_ptr_(new pcl::NormalDistributionsTransform<CloudData::POINT,
                                                     CloudData::POINT>()) {
  setRegistrationParam(res, step_size, trans_eps, max_iter);
}

bool NDTRegistration::setRegistrationParam(double res, double step_size,
                                           double trans_eps, int max_iter) {
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
                                const Eigen::Matrix4d &predict_pose,
                                CloudData::CLOUD_PTR &filtered_cloud_ptr,
                                Eigen::Matrix4d &result_pose) {
  ndt_ptr_->setInputSource(input_source);
  ndt_ptr_->align(*filtered_cloud_ptr, predict_pose.cast<float>());
  result_pose = ndt_ptr_->getFinalTransformation().cast<double>();
  return true;
}

double NDTRegistration::getFitnessScore() { return ndt_ptr_->getFitnessScore(); }

} // namespace avp_mapping
