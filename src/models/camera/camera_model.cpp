/*
 * @Author: ding.yin
 * @Date: 2022-10-17 11:07:06
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:05:45
 */

#include "models/camera/camera_model.hpp"

namespace avp_mapping {

CameraModel::CameraModel(YAML::Node &node) {
  fx_ = node["fx"].as<float>();
  fy_ = node["fy"].as<float>();
  cx_ = node["cx"].as<float>();
  cy_ = node["cy"].as<float>();
  K_(0, 0) = fx_;
  K_(1, 1) = fy_;
  K_(0, 2) = cx_;
  K_(1, 2) = cy_;
  K_(2, 2) = 1.0f;
}

CameraModel::CameraModel(float fx, float fy, float cx, float cy)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
  K_(0, 0) = fx_;
  K_(1, 1) = fy_;
  K_(0, 2) = cx_;
  K_(1, 2) = cy_;
  K_(2, 2) = 1.0f;
}

bool CameraModel::img2BevCloud(ImageData &img_input,
                               CloudData::CLOUD_PTR &bev_cloud_output,
                               Eigen::Matrix4f &camera_to_base) {

  return true;
}

bool img2BevImage(ImageData &img_input, ImageData &img_output,
                  Eigen::Matrix4f &camera_to_base, float scale) {

  
  return true;
}

bool imgSegmentation(ImageData &img_raw, ImageData &img_segmented) {

  

  return true;
}

} // namespace avp_mapping