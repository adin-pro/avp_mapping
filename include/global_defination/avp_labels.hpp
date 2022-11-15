/*
 * @Author: ding.yin
 * @Date: 2022-10-02 13:39:31
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-18 10:15:31
 */
#ifndef _AVP_LABELS_H_
#define _AVP_LABELS_H_

#include <opencv2/opencv.hpp>
#include <vector>
#include <Eigen/Dense>

namespace avp_mapping {
enum AVPLabels {
  ROAD = 0,
  LANE_LINE = 1,
  PARKING_LINE = 2,
  MARKER = 3,
  CROSSING = 4,
  VEHICLE = 5,
  BACKGROUND = 6,
};

extern std::vector<Eigen::Vector3d> AVPColors;

}; // namespace avp_mapping

#endif // _AVP_LABELS_H_