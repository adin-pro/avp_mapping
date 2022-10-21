/*
 * @Author: ding.yin
 * @Date: 2022-10-17 20:46:35
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 20:52:50
 */

#include "global_defination/avp_labels.hpp"

namespace avp_mapping {
std::vector<cv::Scalar> AVPColors{
    cv::Scalar(0, 0, 0),      // ROAD
    cv::Scalar(245, 45, 190), // LANE_LINE
    cv::Scalar(65, 125, 255), // PARKING _LINE
    cv::Scalar(255, 255, 0),  // MARKER
    cv::Scalar(0, 0, 0),      // CROSSING
    cv::Scalar(0, 0, 0),      // VEHICLE
    cv::Scalar(0, 0, 0),      // BACKGROUND
};
}