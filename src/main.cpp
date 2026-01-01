/**
 * @file main.cpp
 * @brief Launches the WebRTC bridge ROS2 node.
 * @author Mark West (mark@telemething.com)
 */

#include <rclcpp/rclcpp.hpp>
#include "webrtc_bridge/webrtc_bridge_node.hpp"

/**
 * @brief Application entry point that initializes ROS2 and spins the WebRTC bridge node.
 */
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<webrtc_bridge::WebRTCBridgeNode>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}
