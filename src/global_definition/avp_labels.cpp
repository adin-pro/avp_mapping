/*
 * @Author: ding.yin
 * @Date: 2022-10-17 20:46:35
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-10-17 20:52:50
 */

#include "global_defination/avp_labels.hpp"

namespace avp_mapping {
std::vector<Eigen::Vector3d> AVPColors{
    Eigen::Vector3d(0, 0, 0),      // ROAD
    Eigen::Vector3d(245, 45, 190), // LANE_LINE
    Eigen::Vector3d(65, 125, 255), // PARKING _LINE
    Eigen::Vector3d(255, 255, 0),  // MARKER
    Eigen::Vector3d(79, 255, 255),      // CROSSING
    Eigen::Vector3d(0, 0, 0),      // VEHICLE
    Eigen::Vector3d(178, 178, 178),      // SKY
};
}