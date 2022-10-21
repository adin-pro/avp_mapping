/*
 * @Author: ding.yin
 * @Date: 2022-10-17 11:07:06
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 20:43:38
 */

#include "models/camera/camera_model.hpp"
#include "glog/logging.h"
#include <iostream>
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
                               Eigen::Matrix4f &base2cam) {
  // intrinsic
  Eigen::Matrix3f K_inv = K_.inverse();
  //extrinsic
  Eigen::Matrix3f cam2base;
  cam2base << base2cam(0, 0), base2cam(0, 1), base2cam(0, 3), base2cam(1, 0),
      base2cam(1, 1), base2cam(1, 3), base2cam(2, 0), base2cam(2, 1),
      base2cam(2, 3);
  Eigen::Matrix3f cam2base_inv = cam2base.inverse();
  Eigen::Matrix3f axis_trans =
      (Eigen::Matrix3f() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
  // iteration
  cv::Mat image = img_input.image.clone();  
  int rows = image.rows;
  int cols = image.cols;
  for (int v = 0; v < rows; v += 10) {
    // row pointer
    const uchar *ptr = image.ptr<uchar>(v);
    for (int u = 0; u < cols; u += 10) {
      // bgr color encoding
      int b = ptr[3 * u];
      int g = ptr[3 * u + 1];
      int r = ptr[3 * u + 2];
      // point of pixel
      Eigen::Vector3f pp;
      pp(0) = u;
      pp(1) = v;
      pp(2) = 1.0;
      // point in camera coordinate
      Eigen::Vector3f pc = K_inv * pp;
      // axis transfromation
      pc = axis_trans * pc;
      // point in world coordinate
      Eigen::Vector3f pw = cam2base_inv * pc;
      // convert pw to homogeneous coordinates
      pw(0) /= pw(2);
      pw(1) /= pw(2);
      // remove low reliable points
      double distance = sqrt(pw.x() * pw.x() + pw.y() * pw.y());
      if (distance > 10.0)
        continue;
      CloudData::POINT cloud_point;
      cloud_point.x = pw.x();
      cloud_point.y = pw.y();
      cloud_point.z = 0.0;
      cloud_point.r = r;
      cloud_point.b = b;
      cloud_point.g = g;
      bev_cloud_output->points.push_back(cloud_point);
    }
  }
  return true;
}

bool img2BevImage(ImageData &img_input, ImageData &img_output,
                  Eigen::Matrix4f &camera_to_base, float scale) {

  return true;
}

bool imgSegmentation(ImageData &img_raw, ImageData &img_segmented) {

  return true;
}

Eigen::Matrix3f CameraModel::getIntrinsic() { return K_; }

} // namespace avp_mapping