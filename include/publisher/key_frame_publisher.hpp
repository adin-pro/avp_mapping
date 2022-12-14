/*
 * @Author: Ren Qian
 * @Date: 2020-02-06 21:05:47
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 15:40:25
 */
#ifndef _KEY_FRAME_PUBLISHER_H_
#define _KEY_FRAME_PUBLISHER_H_

#include <string>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/ros.h>

#include <sensor_data/key_frame.hpp>

namespace avp_mapping {

class KeyFramePublisher {

public:
  KeyFramePublisher(ros::NodeHandle &nh, std::string topic_name,
                    std::string frame_id, size_t buff_size);
  KeyFramePublisher() = default;

  void publish(KeyFrame &keyframe_input);

  bool hasSubscriber();

private:
  ros::NodeHandle nh_;
  ros::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _KEY_FRAME_PUBLISHER_H_