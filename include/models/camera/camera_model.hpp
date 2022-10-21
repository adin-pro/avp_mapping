/*
 * @Author: ding.yin
 * @Date: 2022-10-17 10:41:43
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 20:43:33
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

  CameraModel(YAML::Node &node);

  CameraModel(float fx, float fy, float cx, float cy);

  bool img2BevCloud(ImageData &img_input,
                    CloudData::CLOUD_PTR &bev_cloud_output,
                    Eigen::Matrix4f &camera_to_base);

  bool img2BevImage(ImageData &img_input, ImageData &img_output,
                    Eigen::Matrix4f &base2cam, float scale);

  Eigen::Matrix3f getIntrinsic();

private:
  bool imgSegmentation(ImageData &img_raw, ImageData &img_segmented);

  float fx_;
  float fy_;
  float cx_;
  float cy_;
  Eigen::Matrix3f K_ = Eigen::Matrix3f::Identity();
};

} // namespace avp_mapping

#endif // _CAMERA_MODEL_H_