/*
 * @Author: ding.yin
 * @Date: 2022-10-16 16:54:18
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-16 17:05:33
 */

#include "publisher/image_publisher.hpp"

#include "cv_bridge/cv_bridge.h"

namespace avp_mapping {
ImagePublisher::ImagePublisher(ros::NodeHandle &nh, std::string topic_name,
                               std::string frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::Image>(topic_name, buff_size);
}

void ImagePublisher::publish(cv::Mat &img_ptr_input, double time) {
  ros::Time ros_time(static_cast<float>(time));
  publishData(img_ptr_input, ros_time);
}

void ImagePublisher::publish(cv::Mat &img_ptr_input) {
  ros::Time time = ros::Time::now();
  publishData(img_ptr_input, time);
}

void ImagePublisher::publishData(cv::Mat &img_ptr_input, ros::Time time) {
  sensor_msgs::Image img_msg;
  std_msgs::Header header;
  header.stamp = time;
  header.frame_id = frame_id_;
  cv_bridge::CvImage image_bridge = cv_bridge::CvImage(
      header, sensor_msgs::image_encodings::RGB8, img_ptr_input);
  image_bridge.toImageMsg(img_msg);
  publisher_.publish(img_msg);
}

bool ImagePublisher::hasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

} // namespace avp_mapping