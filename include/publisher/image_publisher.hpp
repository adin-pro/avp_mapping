/*
 * @Author: ding.yin
 * @Date: 2022-10-16 16:47:04
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-16 23:27:38
 */
#ifndef _IMAGE_PUBLISHER_H_
#define _IMAGE_PUBLISHER_H_

#include "ros/ros.h"
#include <image_transport/image_transport.h>

#include "sensor_data/image_data.hpp"

namespace avp_mapping {

class ImagePublisher {
public:
  ImagePublisher() = default;
  ImagePublisher(ros::NodeHandle &nh, std::string topic_name,
                 std::string frame_id, size_t buff_size);

  void publish(cv::Mat image_data, double time);
  void publish(cv::Mat image_data);

  bool hasSubscribers();

private:
  void publishData(cv::Mat& image_data, ros::Time time);

private:
  ros::NodeHandle nh_;
  image_transport::Publisher publisher_;
  std::string frame_id_;
};

} // namespace avp_mapping

#endif // _IMAGE_PUBLISHER_H_