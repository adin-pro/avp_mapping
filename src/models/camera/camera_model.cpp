/*
 * @Author: ding.yin
 * @Date: 2022-10-17 11:07:06
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-03 20:42:36
 */

#include <chrono>
#include <iostream>

#include "glog/logging.h"
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
  K_inv_ = K_.inverse();
}

CameraModel::CameraModel(float fx, float fy, float cx, float cy)
    : fx_(fx), fy_(fy), cx_(cx), cy_(cy) {
  K_(0, 0) = fx_;
  K_(1, 1) = fy_;
  K_(0, 2) = cx_;
  K_(1, 2) = cy_;
  K_(2, 2) = 1.0f;
  K_inv_ = K_.inverse();
}

void calCloudFromImage(Eigen::Matrix3d &K, Eigen::Matrix3d &RT,
                       const cv::Mat &image,
                       pcl::PointCloud<CloudData::POINT>::Ptr &cameraCloud) {
  Eigen::Matrix3d KInv = K.inverse();
  Eigen::Matrix3d RTInv = RT.inverse();
  int row = image.rows;
  int col = image.cols;
  auto start_time = std::chrono::system_clock::now();
  for (int i = 0; i < row; i = i + 2) {
    const uchar *p = image.ptr<uchar>(i);
    for (int j = 0; j < col; j = j + 4) {

      int b = p[3 * j];
      int g = p[3 * j + 1];
      int r = p[3 * j + 2];
      //   there,for simplicity,just according to color,detect invalid area like
      //   sky area; for real scene,should adapt machine learning method or
      //   other method detecting invalid area
      if (b == 178) {
        break;
      }

      Eigen::Vector3d u;
      u(0) = j;
      u(1) = i;
      u(2) = 1;
      Eigen::Vector3d u1;

      u1 = KInv * u;
      u1 = RTInv * u1;
      u1(0) = u1.x() / u1.z();
      u1(1) = u1.y() / u1.z();
      double dis = sqrt(u1.x() * u1.x() + u1.y() * u1.y());

      if (dis > 10)
        continue;

      CloudData::POINT po;
      po.x = u1.x();
      po.y = u1.y();
      po.z = 0;
      po.r = r;
      po.g = g;
      po.b = b;
      cameraCloud->push_back(po);
    }
  }
  auto end_time = std::chrono::system_clock::now();
  LOG(INFO) << "transform loop: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          end_time - start_time)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;
}

bool CameraModel::img2BevCloudWrapper(const cv::Mat &img_input,
                                      CloudData::CLOUD_PTR &bev_cloud_output,
                                      const Eigen::Matrix4f &base2cam) {
  // extrinsic
  Eigen::Matrix3d cam2base;
  cam2base << base2cam(0, 0), base2cam(0, 1), base2cam(0, 3), base2cam(1, 0),
      base2cam(1, 1), base2cam(1, 3), base2cam(2, 0), base2cam(2, 1),
      base2cam(2, 3);
  Eigen::Matrix3d cam2base_inv = cam2base.inverse();
  Eigen::Matrix3d K_double = K_.cast<double>();
  calCloudFromImage(K_double, cam2base_inv, img_input, bev_cloud_output);
  return true;
}

bool CameraModel::img2BevCloud(const cv::Mat &img_input,
                               CloudData::CLOUD_PTR &bev_cloud_output,
                               const Eigen::Matrix4f &base2cam) {
  // extrinsic
  Eigen::Matrix3f base2cam_mat33;
  base2cam_mat33 << base2cam(0, 0), base2cam(0, 1), base2cam(0, 3),
      base2cam(1, 0), base2cam(1, 1), base2cam(1, 3), base2cam(2, 0),
      base2cam(2, 1), base2cam(2, 3);
  Eigen::Matrix3f cam2base = base2cam_mat33.inverse();
  // iteration
  int rows = img_input.rows;
  int cols = img_input.cols;
  auto start_time = std::chrono::system_clock::now();
  for (int v = 0; v < rows; v += 2) {
    // row pointer
    const uchar *ptr = img_input.ptr<uchar>(v);
    for (int u = 0; u < cols; u += 4) {
      // bgr color encoding
      int b = ptr[3 * u];
      int g = ptr[3 * u + 1];
      int r = ptr[3 * u + 2];
      // sky color
      if (b == 178)
        break;
      // point of pixel
      Eigen::Vector3f pp;
      pp(0) = u;
      pp(1) = v;
      pp(2) = 1.0;
      // point in world coordinate
      Eigen::Vector3f pw = cam2base * axis_trans_ * K_inv_ * pp;
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
      bev_cloud_output->push_back(cloud_point);
    }
  }
  auto end_time = std::chrono::system_clock::now();
  LOG(INFO) << "transform loop: "
            << double(std::chrono::duration_cast<std::chrono::microseconds>(
                          end_time - start_time)
                          .count()) *
                   std::chrono::microseconds::period::num /
                   std::chrono::microseconds::period::den;
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
                               Eigen::Matrix4f &base2cam, float scale) {

  int rows = bev_img.rows;
  int cols = bev_img.cols;
  int u_max = img.cols;
  int v_max = img.rows;
  Eigen::Matrix3f rot = base2cam.block<3, 3>(0, 0);
  Eigen::Vector3f trans = base2cam.block<3, 1>(0, 3);
  Eigen::Vector3f pw; // world coordinate
  Eigen::Vector3f pc; // camera coordinate
  Eigen::Matrix3f axis_base_2_cam;
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
      bev_img.at<cv::Vec3b>(row, col)[0] = img.at<cv::Vec3b>(v, u)[0];
      bev_img.at<cv::Vec3b>(row, col)[1] = img.at<cv::Vec3b>(v, u)[1];
      bev_img.at<cv::Vec3b>(row, col)[2] = img.at<cv::Vec3b>(v, u)[2];
    }
  }

  return true;
}

bool imgSegmentation(ImageData &img_raw, ImageData &img_segmented) {

  return true;
}

Eigen::Matrix3f CameraModel::getIntrinsic() { return K_; }

} // namespace avp_mapping