/*
 * @Author: ding.yin
 * @Date: 2022-10-17 10:41:43
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-03 18:55:46
 */
#ifndef _CAMERA_MODEL_H_
#define _CAMERA_MODEL_H_

#include "global_defination/avp_labels.hpp"
#include "opencv2/opencv.hpp"
#include "sensor_data/cloud_data.hpp"
#include "sensor_data/image_data.hpp"
#include "yaml-cpp/yaml.h"

#include "Eigen/Dense"

namespace avp_mapping {

class CameraModel {

public:
  CameraModel() = default;

  CameraModel(const YAML::Node &node);

  CameraModel(double fx, double fy, double cx, double cy);

  bool img2BevCloud(const cv::Mat &img_input,
                    CloudData::CLOUD_PTR &bev_cloud_output,
                    CloudData::CLOUD_PTR &bev_cloud_with_height_out,
                    const Eigen::Matrix4d &camera_to_base);

  bool img2BevImage(const cv::Mat &img_input, cv::Mat &img_output,
                    Eigen::Matrix4d &base2cam, double scale = 0.05);

  Eigen::Matrix3d getIntrinsic();

private:
  bool imgSegmentation(ImageData &img_raw, ImageData &img_segmented);

  double fx_;
  double fy_;
  double cx_;
  double cy_;
  Eigen::Matrix3d K_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d K_inv_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d axis_trans_ =
      (Eigen::Matrix3d() << 0, 0, 1, -1, 0, 0, 0, -1, 0).finished();
  double simi_thre_;
  double valid_cloud_range_;
  double semantic_height_;
};

} // namespace avp_mapping

#endif // _CAMERA_MODEL_H_