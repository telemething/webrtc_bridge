/**
 * @file data_channel_handler.cpp
 * @brief Implements bridging between ROS string topics and WebRTC data channels.
 * @author Mark West (mark@telemething.com)
 */

#include "webrtc_bridge/data_channel_handler.hpp"
#include <iostream>

namespace webrtc_bridge
{

DataChannelHandler::DataChannelHandler(rclcpp::Node::SharedPtr node,
                                         const std::string& topic_out,
                                         const std::string& topic_in)
  : node_(node), topic_out_(topic_out), topic_in_(topic_in)
{
  subscription_ = node_->create_subscription<std_msgs::msg::String>(
    topic_out_, 10,
    std::bind(&DataChannelHandler::ros_topic_callback, this, std::placeholders::_1)
  );

  publisher_ = node_->create_publisher<std_msgs::msg::String>(topic_in_, 10);

  RCLCPP_INFO(node_->get_logger(),
              "DataChannelHandler initialized. Subscribing to '%s', publishing to '%s'",
              topic_out_.c_str(), topic_in_.c_str());
}

DataChannelHandler::~DataChannelHandler()
{
}

void DataChannelHandler::set_send_callback(SendToWebRTCCallback callback)
{
  send_callback_ = std::move(callback);
}

void DataChannelHandler::ros_topic_callback(const std_msgs::msg::String::SharedPtr msg)
{
  if (send_callback_) {
    send_callback_(CHANNEL_ROS_OUT, msg->data);
  }
}

void DataChannelHandler::on_webrtc_message(const std::string& channel_label,
                                            const std::string& message)
{
  if (channel_label == CHANNEL_ROS_IN) {
    publish_to_ros(message);
  }
}

void DataChannelHandler::publish_to_ros(const std::string& message)
{
  auto msg = std_msgs::msg::String();
  msg.data = message;
  publisher_->publish(msg);

  RCLCPP_DEBUG(node_->get_logger(), "Published message to ROS: %s", message.c_str());
}

}  // namespace webrtc_bridge
