/*
 * @Author: ding.yin
 * @Date: 2022-11-12 15:12:56
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 21:40:02
 */

#include <fstream>

#include "glog/logging.h"
#include "opencv2/opencv.hpp"
#include "pcl/common/transforms.h"
#include "pcl/io/pcd_io.h"
#include "ros/ros.h"

#include "publisher/cloud_publisher.hpp"
#include "subscriber/cloud_subscriber.hpp"
#include "subscriber/image_subscriber.hpp"
#include "subscriber/odometry_subscriber.hpp"
#include "tools/file_manager.hpp"

using namespace avp_mapping;

int main(int argc, char **argv) {
  ros::init(argc, argv, "avp_auxiliary_node");
  ros::NodeHandle nh;
  ros::Rate rate(100);
  if (argc < 2) {
    LOG(ERROR) << "Add path of file directory";
  }
  std::string work_path = std::string(argv[1]);
  if (!FileManager::CreateDirectory(work_path + "/slam_data/")) {
    return false;
  }
  if (!FileManager::CreateDirectory(work_path + "/slam_data/image/")) {
    return false;
  }
  if (!FileManager::CreateDirectory(work_path + "/slam_data/cloud/")) {
    return false;
  }
  if (!FileManager::CreateDirectory(work_path + "/slam_data/submap/")) {
    return false;
  }

  std::ofstream out(work_path + "/slam_data/save_data.txt", std::ios::out);

  std::shared_ptr<CloudSubscriber> cloud_sub_ptr =
      std::make_shared<CloudSubscriber>(nh, "/bev/rgb_cloud", 1000);
  std::shared_ptr<ImageSubscriber> image_sub_ptr =
      std::make_shared<ImageSubscriber>(nh, "/bev/image", 1000);
  std::shared_ptr<OdometrySubscriber> odom_sub_ptr =
      std::make_shared<OdometrySubscriber>(nh, "/odom", 1000);

  std::shared_ptr<CloudPublisher> cloud_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "/map", "/map", 1000);

  std::shared_ptr<CloudPublisher> sub_cloud_pub_ptr =
      std::make_shared<CloudPublisher>(nh, "/sub_map", "/map", 1000);

  std::deque<CloudData> cloud_deque;
  std::deque<ImageData> image_deque;
  std::deque<PoseData> odom_deque;

  PoseData last_odom;
  PoseData curr_odom;
  CloudData curr_cloud;
  ImageData curr_image;
  CloudData map_cloud;
  CloudData sub_map;
  PoseData last_key_frame_odom;

  std::deque<CloudData> sub_map_cloud_deque;
  std::deque<PoseData> sub_map_pose_deque;
  std::deque<PoseData> key_frame_pose_deque;

  int sub_map_cnt = 0;
  int num_cloud_data_for_pop = -1;

  while (ros::ok()) {
    static bool inited = false;
    static int keyframe_cnt = 0;
    ros::spinOnce();
    cloud_sub_ptr->parseData(cloud_deque);
    image_sub_ptr->parseData(image_deque);
    odom_sub_ptr->parseData(odom_deque);

    if (!CloudData::controlDuration(cloud_deque, 5.0)) {
      continue;
    }
    if (!ImageData::controlDuration(image_deque, 5.0)) {
      continue;
    }
    if (!PoseData::controlDuration(odom_deque, 5.0)) {
      continue;
    }
    // one iter
    double cloud_ts = cloud_deque.front().time;
    if (!ImageData::getImageDataByTS(image_deque, cloud_ts, curr_image)) {
      continue;
    }
    if (!PoseData::getPoseDataByTS(odom_deque, cloud_ts, curr_odom)) {
      continue;
    }
    curr_cloud = cloud_deque.front();
    if (!inited) {
      inited = true;
      last_odom = curr_odom;
      last_key_frame_odom = curr_odom;
      CloudData::CLOUD_PTR transformed_cloud(new CloudData::CLOUD());
      pcl::transformPointCloud(*curr_cloud.cloud_ptr, *transformed_cloud,
                               curr_odom.pose);
      *map_cloud.cloud_ptr += *transformed_cloud;
    }

    if (PoseData::isFarEnough(last_odom, curr_odom, 0.2)) {
      sub_map_cloud_deque.push_back(curr_cloud);
      sub_map_pose_deque.push_back(curr_odom);
      last_odom = curr_odom;
      sub_map_cnt++;
    }

    if (PoseData::isFarEnough(last_key_frame_odom, curr_odom, 2.0)) {
      // key_frame
      keyframe_cnt++;
      // save image
      cv::imwrite(work_path + "/slam_data/image/" +
                      std::to_string(keyframe_cnt) + ".png",
                  curr_image.image);
      // save cloud
      curr_cloud = cloud_deque.front();
      pcl::io::savePCDFileBinary(work_path + "/slam_data/cloud/" +
                                     std::to_string(keyframe_cnt) + ".pcd",
                                 *curr_cloud.cloud_ptr);
      // global_map
      CloudData::CLOUD_PTR transformed_cloud(new CloudData::CLOUD());
      pcl::transformPointCloud(*curr_cloud.cloud_ptr, *transformed_cloud,
                               curr_odom.pose);
      *map_cloud.cloud_ptr += *transformed_cloud;
      cloud_pub_ptr->publish(map_cloud.cloud_ptr);

      // sub_map
      if (num_cloud_data_for_pop == -1) {
        num_cloud_data_for_pop = sub_map_cnt;
        sub_map_cnt = 0;
      } else {
        // clear sub map
        sub_map.cloud_ptr->clear();
        CloudData::CLOUD_PTR transformed_sub_cloud(new CloudData::CLOUD());
        for (int si = 0; si < sub_map_cloud_deque.size(); si++) {
          transformed_sub_cloud->clear();
          LOG(INFO) << sub_map_cloud_deque[si].time;
          pcl::transformPointCloud(*sub_map_cloud_deque[si].cloud_ptr,
                                   *transformed_sub_cloud,
                                   sub_map_pose_deque[si].pose);
          *sub_map.cloud_ptr += *transformed_sub_cloud;
        }
        while (num_cloud_data_for_pop--) {
          sub_map_cloud_deque.pop_front();
          sub_map_pose_deque.pop_front();
        }
        num_cloud_data_for_pop = sub_map_cnt;
        sub_map_cnt = 0;
        pcl::io::savePCDFileBinary(work_path + "/slam_data/submap/" +
                                       std::to_string(keyframe_cnt-1) + ".pcd",
                                   *sub_map.cloud_ptr);
        sub_cloud_pub_ptr->publish(sub_map.cloud_ptr);
      }

      // save odom
      key_frame_pose_deque.push_back(curr_odom);
      auto q = curr_odom.getQuaternion();
      out << curr_odom.time << " " << curr_odom.pose(0, 3) << " "
          << curr_odom.pose(1, 3) << " " << curr_odom.pose(2, 3) << " " << q.x()
          << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
      last_key_frame_odom = curr_odom;
      LOG(INFO) << "KeyFrame " << keyframe_cnt
                << " saved. Timestamp: " << curr_odom.time;
    }
    cloud_deque.pop_front();
    rate.sleep();
  }

  return 0;
}