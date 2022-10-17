/*
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 19:09:02
 */
#include "sensor_data/imu_data.hpp"

namespace avp_mapping {
Eigen::Matrix3f IMUData::getOrientationMatrix() {
  Eigen::Quaterniond q(orientation.x, orientation.y, orientation.z,
                       orientation.w);
  Eigen::Matrix3f matrix = q.matrix().cast<float>();
  return matrix;
}

bool IMUData::syncData(std::deque<IMUData> &unsyncedData,
                       std::deque<IMUData> &syncedData, double sync_time) {
  while (unsyncedData.size() >= 2) {
    if (unsyncedData.front().time > sync_time)
      return false;
    if (unsyncedData.at(1).time < sync_time) {
      unsyncedData.pop_front();
      continue;
    }
    if (sync_time - unsyncedData.front().time > 0.2) {
      unsyncedData.pop_front();
      break;
    }
    if (unsyncedData.at(1).time - sync_time > 0.2) {
      unsyncedData.pop_front();
      break;
    }
    break;
  }
  if (unsyncedData.size() < 2)
    return false;

  IMUData front_data = unsyncedData.at(0);
  IMUData back_data = unsyncedData.at(1);
  IMUData synced_data;

  double front_scale = (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale = (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;
  
  synced_data.linear_acceleration.x = front_data.linear_acceleration.x * front_scale + back_data.linear_acceleration.x * back_scale;
  synced_data.linear_acceleration.y = front_data.linear_acceleration.y * front_scale + back_data.linear_acceleration.y * back_scale;
  synced_data.linear_acceleration.z = front_data.linear_acceleration.z * front_scale + back_data.linear_acceleration.z * back_scale;

  synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale + back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale + back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale + back_data.angular_velocity.z * back_scale;

  synced_data.orientation.x = front_data.orientation.x * front_scale + back_data.orientation.x * back_scale;
  synced_data.orientation.y = front_data.orientation.y * front_scale + back_data.orientation.y * back_scale;
  synced_data.orientation.z = front_data.orientation.z * front_scale + back_data.orientation.z * back_scale;
  synced_data.orientation.w = front_data.orientation.w * front_scale + back_data.orientation.w * back_scale;

  synced_data.orientation.normlize();
  syncedData.push_back(synced_data);
}

} // namespace avp_mapping
