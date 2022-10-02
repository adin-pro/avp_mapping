/*
 * @Author: ding.yin
 * @Date: 2022-10-02 13:39:31
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-02 14:07:01
 */
#ifndef _AVP_LABELS_H_
#define _AVP_LABELS_H_

#include <opencv2/opencv.hpp>
#include <vector>

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

std::vector<cv::Scalar> AVPColors{
    cv::Scalar(0, 0, 0),      // ROAD
    cv::Scalar(245, 45, 190), // LANE_LINE
    cv::Scalar(65, 125, 255), // PARKING _LINE
    cv::Scalar(255, 255, 0),  // MARKER
    cv::Scalar(0, 0, 0),      // CROSSING
    cv::Scalar(0, 0, 0),      // VEHICLE
    cv::Scalar(0, 0, 0),      // BACKGROUND
};

}; // namespace avp_mapping

#endif // _AVP_LABELS_H_