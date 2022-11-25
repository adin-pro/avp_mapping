/*
 * @Author: ding.yin
 * @Date: 2022-11-21 15:15:45
 * @Last Modified by: ding.yin
 * @Last Modified time: 2022-11-21 15:16:06
 */

#include "models/loop_close/semantic_node.hpp"

#include <iostream>

namespace avp_mapping {

const std::string SemanticNode::getTypeStr() {
  std::string label = "";
  if (type_ == AVPLabels::PARKING_LINE) {
    label = "PARKING_LINE";
  } else if (type_ == AVPLabels::MARKER) {
    label = "MARKER";
  } else if (type_ == AVPLabels::CROSSING) {
    label = "CROSSING";
  } else if (type_ == AVPLabels::STOP_LINE) {
    label = "STOP_LINE";
  } else if (type_ == AVPLabels::BUMP) {
    label = "BUMP";
  } else if (type_ == AVPLabels::LANE_LINE) {
    label = "lane_line";
  }
  return label;
}

void SemanticNode::printInfo() {

  std::cout << "Node " << getTypeStr() << " (px, py, witdth, height) " << px_
            << " " << py_ << " " << width_ << " " << height_ << std::endl;
  for (int i = 0; i < dist_to_other_nodes_.size(); ++i) {
    std::cout << " " << dist_to_other_nodes_[i];
  }
  std::cout << std::endl;
}

} // namespace avp_mapping