/*
 * @Author: ding.yin
 * @Date: 2022-11-05 21:17:48
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-06 16:09:28
 */

#include "mapping/front_end/front_end_flow.hpp"

namespace avp_mapping {

FrontEndFlow::FrontEndFlow(ros::NodeHandle &nh, const std::string &cloud_topic,
                           const std::string &odom_topic) {
  cloud_sub_ptr_ = std::make_shared<CloudSubscriber>(nh, cloud_topic, 1000);
  odom_pub_ptr_ = std::make_shared<OdometryPublisher>(nh, odom_topic, "/base_link",
                                                      "/lidar", 1000);
  front_end_ptr_ = std::make_shared<FrontEnd>(2.0, 20);
}

bool FrontEndFlow::run() {
  if (!readData()) {
    return false;
  }
  if (hasData() && validData()) {
    updateLaserOdometry();
    publishData();
    return true;
  }
  return false;
}

bool FrontEndFlow::readData() {
  cloud_sub_ptr_->parseData(cloud_data_buffer_);
  return true;
}

bool FrontEndFlow::hasData() { return cloud_data_buffer_.size() > 0; }

bool FrontEndFlow::validData() { return true; }

bool FrontEndFlow::updateLaserOdometry() {
  static bool odom_inited = false;
  current_cloud_data_ = cloud_data_buffer_.front();
  cloud_data_buffer_.pop_front();
  if (!odom_inited) {
    odom_inited = true;
    front_end_ptr_->setInitPose(Eigen::Matrix4f::Identity());
  }
  return front_end_ptr_->update(current_cloud_data_, laser_odometry_);
}

bool FrontEndFlow::publishData() {
  odom_pub_ptr_->publish(laser_odometry_, current_cloud_data_.time);
  return true;
}

} // namespace avp_mapping
