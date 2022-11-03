/*
 * @Author: ding.yin
 * @Date: 2022-10-16 16:30:46
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-16 16:39:27
 */
#include "subscriber/image_subscriber.hpp"

namespace avp_mapping {

ImageSubscriber::ImageSubscriber(ros::NodeHandle &nh, std::string topic_name,
                                 size_t buff_size)
    : nh_(nh) {
  subscriber_ = nh_.subscribe(topic_name, buff_size,
                              &ImageSubscriber::msg_callback, this);
}

void ImageSubscriber::msg_callback(const sensor_msgs::ImageConstPtr& image_msg_ptr) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  ImageData img_data;
  img_data.time = image_msg_ptr->header.stamp.toSec();
  img_data.image = cv_bridge::toCvCopy(image_msg_ptr)->image;
  latest_deque_image_data_.push_back(img_data);
}

void ImageSubscriber::parseData(std::deque<ImageData>& image_data_buff) {
  std::unique_lock<std::mutex> ulock(buff_mutex_);
  if (latest_deque_image_data_.size() > 0) {
    image_data_buff.insert(image_data_buff.end(), latest_deque_image_data_.begin(), latest_deque_image_data_.end());
    latest_deque_image_data_.clear();
  }

}


} // namespace avp_mapping