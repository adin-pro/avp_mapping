/*
 * @Author: Ren Qian
 * @Date: 2020-02-28 18:50:16
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-12 21:33:33
 */

#include "sensor_data/pose_data.hpp"
#include "glog/logging.h"

#include <deque>

namespace avp_mapping {

Eigen::Quaterniond PoseData::getQuaternion() {
  Eigen::Quaterniond q;
  q = pose.block<3, 3>(0, 0);
  return q;
}

bool PoseData::syncData(std::deque<PoseData> &unsyncedData,
                        std::deque<PoseData> &syncedData, double sync_time) {
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

  PoseData front_data = unsyncedData.at(0);
  PoseData back_data = unsyncedData.at(1);
  PoseData synced_data;

  double front_scale =
      (back_data.time - sync_time) / (back_data.time - front_data.time);
  double back_scale =
      (sync_time - front_data.time) / (back_data.time - front_data.time);
  synced_data.time = sync_time;

  synced_data.pose(0, 3) =
      front_data.pose(0, 3) * front_scale + back_data.pose(0, 3) * back_scale;
  synced_data.pose(1, 3) =
      front_data.pose(1, 3) * front_scale + back_data.pose(1, 3) * back_scale;
  synced_data.pose(2, 3) =
      front_data.pose(2, 3) * front_scale + back_data.pose(2, 3) * back_scale;
  Eigen::Quaterniond qf = front_data.getQuaternion();
  Eigen::Quaterniond qb = back_data.getQuaternion();
  Eigen::Quaterniond q;
  q.x() = qf.x() * front_scale + qb.x() * back_scale;
  q.y() = qf.y() * front_scale + qb.y() * back_scale;
  q.z() = qf.z() * front_scale + qb.z() * back_scale;
  q.w() = qf.w() * front_scale + qb.w() * back_scale;
  synced_data.pose.block<3, 3>(0, 0) = q.toRotationMatrix();
  syncedData.push_back(synced_data);
  return true;
}

bool PoseData::syncDataWithoutInterpolation(std::deque<PoseData> &unsyncedData,
                                            double sync_time) {
  while (unsyncedData.size() > 0) {
    PoseData pose_data = unsyncedData.front();
    if (pose_data.time + 0.015 < sync_time) {
      unsyncedData.pop_front();
    } else {
      return true;
    }
  }
  return false;
}

bool PoseData::controlDuration(std::deque<PoseData> &pose_deque,
                               double duration) {
  if (pose_deque.size() < 2) {
    return false;
  }
  while (pose_deque.back().time - pose_deque.front().time > duration) {
    pose_deque.pop_front();
  }
  return true;
}

bool PoseData::getPoseDataByTS(std::deque<PoseData> &data_deque,
                               double timestamp, PoseData &result) {
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

bool PoseData::isFarEnough(const PoseData &last_pose, const PoseData &curr_pose,
                           double thre) {
  double dist = fabs(last_pose.pose(0, 3) - curr_pose.pose(0, 3)) +
                fabs(last_pose.pose(1, 3) - curr_pose.pose(1, 3)) +
                fabs(last_pose.pose(2, 3) - curr_pose.pose(2, 3));
  return dist > thre;
}

void PoseData::printPos() {
  LOG(INFO) << "x: " << pose(0, 3) << " y: " << pose(1, 3)
            << " z: " << pose(2, 3);
}

} // namespace avp_mapping
