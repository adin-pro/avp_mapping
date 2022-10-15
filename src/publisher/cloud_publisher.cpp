/*
 * @Author: ding.yin
 * @Date: 2022-10-10 15:21:54
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-10 15:32:01
 */

#include "pcl_conversions/pcl_conversions.h"

#include "publisher/cloud_publisher.hpp"

namespace avp_mapping {
CloudPublisher::CloudPublisher(ros::NodeHandle &nh, std::string topic_name,
                               std::string frame_id, size_t buff_size)
    : nh_(nh), frame_id_(frame_id) {
  publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(topic_name, buff_size);
}

void CloudPublisher::publish(CloudData::CLOUD_PTR &cloud_ptr_input,
                             double time) {
  ros::Time ros_time(static_cast<float>(time));
  publishData(cloud_ptr_input, ros_time);
}

void CloudPublisher::publish(CloudData::CLOUD_PTR &cloud_ptr_input) {
  ros::Time time = ros::Time::now();
  publishData(cloud_ptr_input, time);
}

void CloudPublisher::publishData(CloudData::CLOUD_PTR &cloud_ptr_input,
                                 ros::Time time) {
  sensor_msgs::PointCloud2Ptr cloud_ptr_output(new sensor_msgs::PointCloud2());
  pcl::toROSMsg(*cloud_ptr_input, *cloud_ptr_output);
  cloud_ptr_output->header.stamp = time;
  cloud_ptr_output->header.frame_id = frame_id_;
  publisher_.publish(*cloud_ptr_output);
}

bool CloudPublisher::hasSubscribers() {
  return publisher_.getNumSubscribers() != 0;
}

} // namespace avp_mapping
