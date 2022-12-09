/*
 * @Author: Ren Qian
 * @Date: 2020-02-23 22:20:41
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-05 22:07:25
 */
#include "sensor_data/imu_data.hpp"

namespace avp_mapping {
Eigen::Matrix3d IMUData::getOrientationMatrix() {
  Eigen::Quaterniond q(orientation.x, orientation.y, orientation.z,
                       orientation.w);
  Eigen::Matrix3d matrix = q.matrix();
  return matrix;
}

bool IMUData::ControlDuration(std::deque<IMUData> &imu_deque, double duration) {
  if (imu_deque.size() < 2) {
    return false;
  }
  while (imu_deque.back().time - imu_deque.front().time > duration) {
    imu_deque.pop_front();
  }
  return true;
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

  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;

  synced_data.linear_acceleration.x =
      front_data.linear_acceleration.x * front_scale +
      back_data.linear_acceleration.x * back_scale;
  synced_data.linear_acceleration.y =
      front_data.linear_acceleration.y * front_scale +
      back_data.linear_acceleration.y * back_scale;
  synced_data.linear_acceleration.z =
      front_data.linear_acceleration.z * front_scale +
      back_data.linear_acceleration.z * back_scale;

  synced_data.angular_velocity.x = front_data.angular_velocity.x * front_scale +
                                   back_data.angular_velocity.x * back_scale;
  synced_data.angular_velocity.y = front_data.angular_velocity.y * front_scale +
                                   back_data.angular_velocity.y * back_scale;
  synced_data.angular_velocity.z = front_data.angular_velocity.z * front_scale +
                                   back_data.angular_velocity.z * back_scale;

  synced_data.orientation.x = front_data.orientation.x * front_scale +
                              back_data.orientation.x * back_scale;
  synced_data.orientation.y = front_data.orientation.y * front_scale +
                              back_data.orientation.y * back_scale;
  synced_data.orientation.z = front_data.orientation.z * front_scale +
                              back_data.orientation.z * back_scale;
  synced_data.orientation.w = front_data.orientation.w * front_scale +
                              back_data.orientation.w * back_scale;

  synced_data.orientation.normlize();
  syncedData.push_back(synced_data);
  return true;
}

bool IMUData::getIMUDataByTS(std::deque<IMUData> &data_deque,
                               double timestamp, IMUData &result) {
  int data_size = data_deque.size();
  if (data_size == 0) {
    LOG(WARNING) << "No data in the deque";
    return false;
  }
  if (data_size == 1) {
    double ts_diff = timestamp - data_deque.back().time;
    // timestamp diff should be lower 1/freq 30Hz
    if (ts_diff > 0.04) {
      LOG(WARNING) << "Only one data in the deque, timestamp diff is too large "
                   << timestamp << " " << data_deque.back().time;
      return false;
    }
    result = data_deque.back();
  } else if (timestamp >= data_deque.back().time) {
    double ts_diff = timestamp - data_deque.back().time;
    // timestamp diff should be lower 1/freq 30Hz
    if (ts_diff > 0.04) {
      LOG(WARNING) << "Too large time diff " << timestamp << " "
                   << data_deque.back().time;
      return false;
    }
    result = data_deque.back();
  } else if (timestamp <= data_deque.front().time) {
    double ts_diff = data_deque.front().time - timestamp;
    // timestamp diff should be lower 1/freq 30Hz
    if (ts_diff > 0.04) {
      LOG(WARNING) << "Too large time diff " << timestamp << " "
                   << data_deque.front().time;
      return false;
    }
    result = data_deque.front();
  } else {
    // iteration
    int first_index = -1;
    int second_index = -1;
    for (size_t i = 0; i < data_deque.size() - 1; ++i) {
      if (data_deque[i].time > timestamp) {
        continue;
      } else {
        // data_deque[i].time <= timestamp
        if (data_deque[i + 1].time >= timestamp) {
          first_index = i;
          second_index = i + 1;
          break;
        }
      }
    }
    if (first_index == -1) {
      LOG(ERROR) << "Can not find right timestamp! Deque front_ts: "
                 << data_deque.front().time << " back_ts "
                 << data_deque.back().time << " timestamp " << timestamp;
    }
    result = data_deque[second_index];
  }

  return true;
}

} // namespace avp_mapping
