/*
 * @Author: ding.yin
 * @Date: 2022-10-05 18:27:57
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-05 18:49:06
 */
#ifndef _IMAGE_SUBSCRIBER_H_
#define _IMAGE_SUBSCRIBER_H_

#include <deque>
#include <mutex>
#include <string>

#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


namespace avp_mapping {

class ImageSubscriber {
public:
  ImageSubscriber() = default;
  ImageSubscriber(ros::NodeHandle &nh_, std::string topic_name,
                  size_t buff_size);
  void parseData(std::deque<cv::Mat> &deque_image_data);

private:
  void msg_callback(const sensor_msgs::ImageConstPtr &img_msg_ptr);

private:
  ros::NodeHandle nh_;
  ros::Subscriber subscriber_;
  std::deque<cv::Mat> latest_deque_image_data_;

  std::mutex buff_mutex_;
};

} // namespace avp_mapping

#endif // _IMAGE_SUBSCRIBER_H_