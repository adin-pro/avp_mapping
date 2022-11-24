/*
 * @Author: ding.yin
 * @Date: 2022-10-17 11:07:06
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-06 14:29:05
 */

#include <chrono>
#include <iostream>

#include "global_defination/avp_labels.hpp"
#include "glog/logging.h"
#include "models/camera/camera_model.hpp"
#include "yaml-cpp/yaml.h"

namespace avp_mapping {

CameraModel::CameraModel(const YAML::Node &node) {
  fx_ = node["fx"].as<double>();
  fy_ = node["fy"].as<double>();
  cx_ = node["cx"].as<double>();
  cy_ = node["cy"].as<double>();
  simi_thre_ = node["similarity_thre"].as<double>();
  valid_cloud_range_ = node["valid_cloud_range"].as<double>();
  semantic_height_ = node["semantic_height"].as<double>();

  K_(0, 0) = fx_;
  K_(1, 1) = fy_;
  K_(0, 2) = cx_;
  K_(1, 2) = cy_;
  K_(2, 2) = 1.0f;
  K_inv_ = K_.inverse();
}

bool CameraModel::img2BevCloud(const cv::Mat &img_input,
                               CloudData::CLOUD_PTR &bev_cloud_output,
                               CloudData::CLOUD_PTR &bev_cloud_with_height_out,
                               const Eigen::Matrix4d &base2cam) {
  // extrinsic
  Eigen::Matrix3d base2cam_mat33;
  base2cam_mat33 << base2cam(0, 0), base2cam(0, 1), base2cam(0, 3),
      base2cam(1, 0), base2cam(1, 1), base2cam(1, 3), base2cam(2, 0),
      base2cam(2, 1), base2cam(2, 3);
  Eigen::Matrix3d cam2base = base2cam_mat33.inverse();
  // iteration
  int rows = img_input.rows;
  int cols = img_input.cols;
  for (int v = 0; v < rows; v += 2) {
    // row pointer
    const uchar *ptr = img_input.ptr<uchar>(v);
    for (int u = 0; u < cols; u += 4) {
      // bgr color encoding
      int r = ptr[3 * u];
      int g = ptr[3 * u + 1];
      int b = ptr[3 * u + 2];
      if (r <= 10 && g <= 10 && b <= 10)
        continue;
      // sky color
      if (b == 178)
        break;
      // point of pixel
      Eigen::Vector3d pp;
      pp(0) = u;
      pp(1) = v;
      pp(2) = 1.0;
      // point in world coordinate
      Eigen::Vector3d pw = cam2base * axis_trans_ * K_inv_ * pp;
      // convert pw to homogeneous coordinates
      pw(0) /= pw(2);
      pw(1) /= pw(2);
      // remove low reliable points
      if (fabsf64(pw.x()) > valid_cloud_range_ ||
          fabsf64(pw.y()) > valid_cloud_range_)
        continue;
      Eigen::Vector3d v_3d(r, g, b);
      double max_similarity = 0.0;
      int semantic_type = 0;
      for (size_t i = 0; i < AVPColors.size(); ++i) {
        double similarity =
            v_3d.dot(AVPColors[i]) / v_3d.norm() / AVPColors[i].norm();
        if (similarity > max_similarity) {
          semantic_type = i;
          max_similarity = similarity;
        }
      }
      if (semantic_type == AVPLabels::ROAD) // ROAD
        continue;
      if (semantic_type == AVPLabels::BACKGROUND) // BACKGROUND
        continue;
      CloudData::POINT cloud_point;
      cloud_point.x = pw.x();
      cloud_point.y = pw.y();
      cloud_point.z = 0.0;
      cloud_point.r = AVPColors[semantic_type].x();
      cloud_point.g = AVPColors[semantic_type].y();
      cloud_point.b = AVPColors[semantic_type].z();
      bev_cloud_output->push_back(cloud_point);
      CloudData::POINT cloud_point_height;
      cloud_point_height.x = pw.x();
      cloud_point_height.y = pw.y();
      cloud_point_height.z = semantic_height_ * semantic_type;
      cloud_point_height.r = AVPColors[semantic_type].x();
      cloud_point_height.g = AVPColors[semantic_type].y();
      cloud_point_height.b = AVPColors[semantic_type].z();
      bev_cloud_with_height_out->push_back(cloud_point_height);
    }
  }
  return true;
}

/*
convert pin-hole image to bird-eye-view image using intrinsic and extrinsic
images
@param img: original image
@param bev_img: bird-eye-view image after IPM
@param base2cam: transformation from base_link to camera_link
@param scale: zoom scale -> m/pixel, default value is 0.05 m/pixel
*/

bool CameraModel::img2BevImage(const cv::Mat &img, cv::Mat &bev_img,
                               Eigen::Matrix4d &base2cam, double scale) {

  int rows = bev_img.rows;
  int cols = bev_img.cols;
  int u_max = img.cols;
  int v_max = img.rows;
  Eigen::Matrix3d rot = base2cam.block<3, 3>(0, 0);
  Eigen::Vector3d trans = base2cam.block<3, 1>(0, 3);
  Eigen::Vector3d pw; // world coordinate
  Eigen::Vector3d pc; // camera coordinate
  Eigen::Matrix3d axis_base_2_cam;
  axis_base_2_cam << 0, -1, 0, 0, 0, -1, 1, 0, 0;

  for (int row = 0; row < rows; ++row) {
    for (int col = 0; col < cols; ++col) {
      pw.x() = (rows / 2 - row) * scale;
      pw.y() = (cols / 2 - col) * scale;
      pw.z() = 0.0;

      pc = rot * pw + trans;
      pc = K_ * axis_base_2_cam * pc;
      int u = pc.x() / pc.z();
      int v = pc.y() / pc.z();
      if (pc.z() < 0 || u < 0 || u >= u_max || v < 0 || v >= v_max)
        continue;

      int r = img.at<cv::Vec3b>(v, u)[0];
      int g = img.at<cv::Vec3b>(v, u)[1];
      int b = img.at<cv::Vec3b>(v, u)[2];
      Eigen::Vector3d v_3d(r, g, b);
      double max_similarity = 0.0;
      int semantic_type = 0;
      for (size_t i = 0; i < AVPColors.size(); ++i) {
        double similarity =
            v_3d.dot(AVPColors[i]) / v_3d.norm() / AVPColors[i].norm();
        if (similarity > max_similarity) {
          semantic_type = i;
          max_similarity = similarity;
          if (similarity > simi_thre_)
            break;
        }
      }
      if (max_similarity < simi_thre_)
        semantic_type = AVPLabels::ROAD;
      if (semantic_type == AVPLabels::BACKGROUND) // BACKGROUND
        continue;
      bev_img.at<cv::Vec3b>(row, col)[0] = AVPColors[semantic_type].z();
      bev_img.at<cv::Vec3b>(row, col)[1] = AVPColors[semantic_type].y();
      bev_img.at<cv::Vec3b>(row, col)[2] = AVPColors[semantic_type].x();
    }
  }

  return true;
}

bool imgSegmentation(ImageData &img_raw, ImageData &img_segmented) {

  return true;
}

Eigen::Matrix3d CameraModel::getIntrinsic() { return K_; }

} // namespace avp_mapping