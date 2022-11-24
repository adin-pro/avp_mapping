/*
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:39:49
 */
#ifndef _ODOMETRY_PUBLISHER_H_
#define _ODOMETRY_PUBLISHER_H_

#include <string>

#include <Eigen/Dense>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

namespace avp_mapping {
class OdometryPublisher {

public:
  OdometryPublisher(ros::NodeHandle &nh, std::string topic_name,
                    std::string base_frame_id, std::string child_frame_id,
                    size_t buff_size);

  OdometryPublisher() = default;

  void publish(const Eigen::Matrix4d &transform_matrix, double time);
  void publish(const Eigen::Matrix4d &transform_matrix);

  bool hasSubscribers();

private:
  void publishData(const Eigen::Matrix4d &transform_matrix, ros::Time time);

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  nav_msgs::Odometry odometry_;
};

} // namespace avp_mapping

#endif // _ODOMETRY_PUBLISHER_H_