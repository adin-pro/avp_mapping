/*
 * @Author: ding.yin
 * @Date: 2022-11-05 21:17:48
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-06 16:09:28
 */

#include "mapping/front_end/front_end_flow.hpp"

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "pcl/io/pcd_io.h"
#include "tools/file_manager.hpp"
#include "yaml-cpp/yaml.h"

namespace avp_mapping {

FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, const std::string &work_dir) {
  YAML::Node config_node =
      YAML::LoadFile(work_dir + "/config/mapping/front_end.yaml");
  // subscribers
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(
      nh, config_node["cloud_sub_topic"].as<std::string>(), 1000);

  cloud_height_sub_ptr_ = std::make_shared<CloudSubscriber>(
      nh, config_node["cloud_height_sub_topic"].as<std::string>(), 1000);

  odom_sub_ptr_ = std::make_shared<OdometrySubscriber>(
      nh, config_node["odom_sub_topic"].as<std::string>(), 1000);

  image_sub_ptr_ = std::make_shared<ImageSubscriber>(
      nh, config_node["image_sub_topic"].as<std::string>(), 1000);

  imu_sub_ptr_ = std::make_shared<IMUSubscriber>(
      nh, config_node["imu_sub_topic"].as<std::string>(), 1000);

  // publishers
  kf_cloud_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, config_node["kf_cloud_pub_topic"].as<std::string>(),
      config_node["frame_id"].as<std::string>(), 100);

  kf_cloud_height_pub_ptr_ = std::make_shared<CloudPublisher>(
      nh, config_node["kf_cloud_height_pub_topic"].as<std::string>(),
      config_node["frame_id"].as<std::string>(), 100);

  kf_image_pub_ptr_ = std::make_shared<ImagePublisher>(
      nh, config_node["kf_image_pub_topic"].as<std::string>(),
      config_node["frame_id"].as<std::string>(), 100);

  kf_pub_ptr_ = std::make_shared<KeyFramePublisher>(
      nh, config_node["kf_pub_topic"].as<std::string>(),
      config_node["frame_id"].as<std::string>(), 100);

  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(
      nh, config_node["odom_pub_topic"].as<std::string>(),
      config_node["odom_pub_parent_frame"].as<std::string>(),
      config_node["odom_pub_child_frame"].as<std::string>(), 1000);

  // front_end_ptr_ = std::make_shared<FrontEnd>(2.0, 20);

  key_frame_dist_ = config_node["key_frame_dist"].as<double>();
  sub_frame_dist_ = config_node["sub_frame_dist"].as<double>();
  control_duration_ = config_node["control_duration"].as<double>();

  // save kfs
  save_kf_ = config_node["save_kf"].as<bool>();
  std::string kf_save_path = config_node["kf_save_path"].as<std::string>();
  kf_cloud_save_path_ = kf_save_path + "/kf_cloud/";
  kf_cloud_height_save_path_ = kf_save_path + "/kf_cloud_height/";
  kf_image_save_path_ = kf_save_path + "/kf_image/";

  FileManager::CreateDirectory(kf_save_path);

  FileManager::CreateDirectory(kf_cloud_save_path_);

  FileManager::CreateDirectory(kf_cloud_height_save_path_);

  FileManager::CreateDirectory(kf_image_save_path_);

  kf_pose_fd_.open(kf_save_path + "/kf_poses.txt", std::ios::out);

  use_imu_wheel_odom_ = config_node["use_imu_wheel_odom"].as<bool>();
  imu_noise_ = config_node["imu_noise"].as<double>();
  imu_bias_ = config_node["imu_bias"].as<double>();

  // add yaw scale
  add_scale_ = config_node["add_scale"].as<bool>();
  yaw_scale_ = config_node["yaw_scale"].as<double>();

  if (use_imu_wheel_odom_) {
    LOG(INFO) << "Use imu-wheel odometry. Imu noise scale " << imu_noise_;
  } else {
    LOG(INFO) << "Use wheel odometry. Yaw scale" << yaw_scale_;
  }

  odom_imu_out_.open(kf_save_path + "/odom_imu.txt", std::ios::out);
}

bool FrontEndFlow::run() {
  if (!readData()) {
    return false;
  }
  if (hasData() && validData()) {
    updateOdometry();
    publishOdom();
    if (HasNewKF()) {
      LOG(INFO) << "Find New KF " << kf_id_;
      if (save_kf_)
        saveData();
      publishKFData();
    }
    return true;
  }
  return false;
}

bool FrontEndFlow::readData() {
  cloud_sub_ptr_->parseData(cloud_data_buffer_);
  cloud_height_sub_ptr_->parseData(cloud_height_data_buffer_);
  image_sub_ptr_->parseData(image_data_buffer_);
  odom_sub_ptr_->parseData(odom_data_buffer_);
  imu_sub_ptr_->parseData(imu_data_buffer_);
  return true;
}

bool FrontEndFlow::hasData() { // sync data
  if (!CloudData::controlDuration(cloud_data_buffer_, control_duration_)) {
    return false;
  }
  if (!CloudData::controlDuration(cloud_data_buffer_, control_duration_)) {
    return false;
  }
  if (!ImageData::controlDuration(image_data_buffer_, control_duration_)) {
    return false;
  }
  if (!PoseData::controlDuration(odom_data_buffer_, control_duration_)) {
    return false;
  }
  if (!IMUData::ControlDuration(imu_data_buffer_, control_duration_)) {
    return false;
  }
  return true;
}

bool FrontEndFlow::validData() {
  curr_cloud_data_ = cloud_data_buffer_.front();
  curr_cloud_height_data_ = cloud_height_data_buffer_.front();
  cloud_data_buffer_.pop_front();
  cloud_height_data_buffer_.pop_front();
  kf_sync_time_ = curr_cloud_data_.time;
  if (!ImageData::getImageDataByTS(image_data_buffer_, kf_sync_time_,
                                   curr_image_data_)) {
    return false;
  }
  if (!PoseData::getPoseDataByTS(odom_data_buffer_, kf_sync_time_,
                                 curr_odom_data_)) {
    return false;
  }
  if (!IMUData::getIMUDataByTS(imu_data_buffer_, kf_sync_time_,
                               curr_imu_data_)) {
    return false;
  }

  odom_imu_out_ << kf_sync_time_ << " " << curr_odom_data_.pose(0, 3) << " "
                << curr_odom_data_.pose(1, 3) << " "
                << curr_odom_data_.velo.linear_velocity.x << " "
                << curr_odom_data_.velo.linear_velocity.y << " "
                << curr_odom_data_.velo.angular_velocity.z << " "
                << curr_imu_data_.angular_velocity.z << std::endl;
  odom_imu_out_.flush();

  return true;
}

bool FrontEndFlow::updateOdometry() {
  // use odom as current pose
  // TODO replace it with encoder-imu fusion odometry
  static bool inited = false;
  if (!inited) {
    inited = true;
    last_odom_data_ = curr_odom_data_;
    curr_frame_pose_ = curr_odom_data_;
    last_frame_pose_ = curr_odom_data_;
    return true;
  }
  if (use_imu_wheel_odom_) {
    Eigen::Vector3d euler_angles =
        last_frame_pose_.pose.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
    double yaw_theta = euler_angles.z();
    double w_imu = curr_imu_data_.angular_velocity.z +
                   imu_noise_ * (static_cast<double>(rand()) /
                                     static_cast<double>(RAND_MAX) -
                                 0.5) +
                   imu_bias_;
    LOG(INFO) << imu_noise_ * (static_cast<double>(rand()) /
                                   static_cast<double>(RAND_MAX) -
                               0.5)
              << std::endl;
    double dt = curr_odom_data_.time - last_odom_data_.time;
    double d_theta = w_imu * dt;
    double v = sqrt(curr_odom_data_.velo.linear_velocity.x *
                        curr_odom_data_.velo.linear_velocity.x +
                    curr_odom_data_.velo.linear_velocity.y *
                        curr_odom_data_.velo.linear_velocity.y);

    curr_frame_pose_.time = curr_odom_data_.time;
    curr_frame_pose_.pose(0, 3) =
        last_frame_pose_.pose(0, 3) - cos(yaw_theta + d_theta / 2.0) * v * dt;
    curr_frame_pose_.pose(1, 3) =
        last_frame_pose_.pose(1, 3) - sin(yaw_theta + d_theta / 2.0) * v * dt;

    yaw_theta += d_theta;
    curr_frame_pose_.pose.block<3, 3>(0, 0) =
        (Eigen::AngleAxisd(euler_angles.x(), Eigen::Vector3d::UnitX()) *
         Eigen::AngleAxisd(euler_angles.y(), Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(yaw_theta, Eigen::Vector3d::UnitZ()))
            .toRotationMatrix();

    last_frame_pose_ = curr_frame_pose_;
    last_odom_data_ = curr_odom_data_;
  } else {
    curr_frame_pose_.time = curr_odom_data_.time;
    Eigen::Matrix4d delta_odom =
        last_odom_data_.pose.inverse() * curr_odom_data_.pose;
    // add yaw scale
    if (add_scale_) {
      Eigen::Vector3d euler_angles =
          delta_odom.block<3, 3>(0, 0).eulerAngles(0, 1, 2);
      delta_odom.block<3, 3>(0, 0) =
          (Eigen::AngleAxisd(euler_angles.x(), Eigen::Vector3d::UnitX()) *
           Eigen::AngleAxisd(euler_angles.y(), Eigen::Vector3d::UnitY()) *
           Eigen::AngleAxisd(euler_angles.z() * yaw_scale_,
                             Eigen::Vector3d::UnitZ()))
              .toRotationMatrix();
    }
    curr_frame_pose_.pose = last_frame_pose_.pose * delta_odom;
    last_odom_data_ = curr_odom_data_;
    last_frame_pose_ = curr_frame_pose_;
  }
  return true;
}

bool FrontEndFlow::HasNewKF() {
  static bool inited = false;
  if (!inited) {
    last_kf_pose_ = curr_frame_pose_;
    inited = true;
    kf_id_++;
    return true;
  }

  if (PoseData::isFarEnough(last_kf_pose_, curr_frame_pose_, key_frame_dist_)) {
    last_kf_pose_ = curr_frame_pose_;
    kf_id_++;
    return true;
  } else {
    return false;
  }
}

bool FrontEndFlow::publishOdom() {
  odom_pub_ptr_->publish(curr_odom_data_.pose, curr_odom_data_.time);
  return true;
}

bool FrontEndFlow::publishKFData() {
  kf_cloud_pub_ptr_->publish(curr_cloud_data_.cloud_ptr, kf_sync_time_);
  LOG(INFO) << "Publish KF Cloud " << curr_cloud_data_.cloud_ptr->points.size();
  kf_cloud_height_pub_ptr_->publish(curr_cloud_height_data_.cloud_ptr,
                                    kf_sync_time_);
  LOG(INFO) << "Publish KF height Cloud "
            << curr_cloud_height_data_.cloud_ptr->points.size();
  kf_image_pub_ptr_->publish(curr_image_data_.image, kf_sync_time_);
  LOG(INFO) << "Publish KF Image " << kf_sync_time_;
  // kf
  KeyFrame kf;
  kf.time = kf_sync_time_;
  kf.pose = curr_frame_pose_.pose;
  kf.index = kf_id_;
  kf_pub_ptr_->publish(kf);
  LOG(INFO) << "kF " << kf_id_ << " x " << kf.pose(0, 3) << " y "
            << kf.pose(1, 3);
  return true;
}

bool FrontEndFlow::saveData() {
  pcl::io::savePCDFileBinary(kf_cloud_save_path_ + std::to_string(kf_id_) +
                                 ".pcd",
                             *curr_cloud_data_.cloud_ptr);

  pcl::io::savePCDFileBinary(kf_cloud_height_save_path_ +
                                 std::to_string(kf_id_) + ".pcd",
                             *curr_cloud_height_data_.cloud_ptr);

  cv::imwrite(kf_image_save_path_ + std::to_string(kf_id_) + ".png",
              curr_image_data_.image);

  auto q = curr_frame_pose_.getQuaternion();
  kf_pose_fd_ << curr_frame_pose_.time << " " << curr_frame_pose_.pose(0, 3)
              << " " << curr_frame_pose_.pose(1, 3) << " "
              << curr_frame_pose_.pose(2, 3) << " " << q.x() << " " << q.y()
              << " " << q.z() << " " << q.w() << std::endl;
  kf_pose_fd_.flush();
  return true;
}

} // namespace avp_mapping
