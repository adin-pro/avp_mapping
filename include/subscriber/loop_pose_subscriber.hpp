/*
 * @Author: ding.yin
 * @Date: 2022-10-05 16:47:54
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 11:14:37
 */
#ifndef _LOOP_POSE_SUBSCRIBER_H_
#define _LOOP_POSE_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "ros/ros.h"

#include "sensor_data/loop_pose.hpp"

namespace avp_mapping {

class LoopPoseSubscriber {
public:
  LoopPoseSubscriber() = default;
  LoopPoseSubscriber(ros::NodeHandle &nh, std::string topic_name,
                     size_t buff_size);
  void parseData(std::deque<LoopPose> &deque_loop_pose);

private:
  void msg_callback(
      const geometry_msgs::PoseWithCovarianceStampedConstPtr &loop_pose_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<LoopPose> latest_loop_pose_;

  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _LOOP_POSE_SUBSCRIBER_H_