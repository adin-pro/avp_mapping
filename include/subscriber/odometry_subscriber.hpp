/*
 * @Author: Ren Qian
 * @Date: 2019-08-19 19:22:17
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:44:24
 */
#ifndef _ODOMETRY_SUBSCRIBER_H_
#define _ODOMETRY_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"

#include "sensor_data/pose_data.hpp"

namespace avp_mapping {
class OdometrySubscriber {

public:
  OdometrySubscriber() = default;
  OdometrySubscriber(ros::NodeHandle &nh_, std::string topic_name,
                     size_t buff_size);
  void parseData(std::deque<PoseData> &deque_pose_data);

private:
  void msg_callback(const nav_msgs::OdometryConstPtr &odom_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<PoseData> latest_deque_pose_data_;

  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _ODOMETRY_SUBSCRIBER_H_