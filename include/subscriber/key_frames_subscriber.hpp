/*
 * @Author: ding.yin
 * @Date: 2022-10-05 16:41:11
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 16:47:14
 */

#ifndef _KEY_FRAMES_SUBSCRIBER_H_
#define _KEY_FRAMES_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>

#include <sensor_data/key_frame.hpp>

namespace avp_mapping {
class KeyFramesSubscriber {

public:
  KeyFramesSubscriber() = default;
  KeyFramesSubscriber(ros::NodeHandle &nh, std::string topic_name,
                      size_t buff_size);
  void parseData(std::deque<KeyFrame> deque_key_frames);

private:
  void
  msg_callback(const geometry_msgs::PoseStampedConstPtr &key_frames_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<KeyFrame> latest_deque_key_frames_;
  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _KEY_FRAMES_SUBSCRIBER_H_