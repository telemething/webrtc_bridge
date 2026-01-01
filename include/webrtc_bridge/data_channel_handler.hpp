#ifndef WEBRTC_BRIDGE__DATA_CHANNEL_HANDLER_HPP_
#define WEBRTC_BRIDGE__DATA_CHANNEL_HANDLER_HPP_

/**
 * @file data_channel_handler.hpp
 * @brief Declares utilities to bridge ROS string topics with WebRTC data channels.
 * @author Mark West (mark@telemething.com)
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <functional>
#include <memory>
#include <string>

namespace webrtc_bridge
{

/**
 * @brief Bridges ROS string topics to WebRTC data channels in both directions.
 */
class DataChannelHandler
{
public:
  using SendToWebRTCCallback = std::function<void(const std::string& channel_label,
                                                   const std::string& message)>;

  /**
   * @brief Construct the handler and wire up ROS publishers/subscribers.
   * @param node Shared ROS2 node used to create publishers and subscriptions.
   * @param topic_out Topic whose messages are forwarded to WebRTC peers.
   * @param topic_in Topic that receives messages from WebRTC peers.
   */
  DataChannelHandler(rclcpp::Node::SharedPtr node,
                     const std::string& topic_out,
                     const std::string& topic_in);

  /**
   * @brief Default destructor.
   */
  ~DataChannelHandler();

  /**
   * @brief Register the callback used to send data to WebRTC channels.
   * @param callback Function invoked with channel label and message payload.
   */
  void set_send_callback(SendToWebRTCCallback callback);

  /**
   * @brief Process an inbound WebRTC data channel message.
   * @param channel_label Data channel label identifying the source.
   * @param message UTF-8 payload from the peer.
   */
  void on_webrtc_message(const std::string& channel_label, const std::string& message);

  /**
   * @brief Publish a received WebRTC message onto the configured ROS topic.
   * @param message Payload to publish.
   */
  void publish_to_ros(const std::string& message);

  static constexpr const char* CHANNEL_ROS_OUT = "ros_topic_out";
  static constexpr const char* CHANNEL_ROS_IN = "ros_topic_in";

private:
  /**
   * @brief Callback invoked when ROS messages arrive for forwarding to WebRTC.
   * @param msg ROS string message to send to peers.
   */
  void ros_topic_callback(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  SendToWebRTCCallback send_callback_;
  std::string topic_out_;
  std::string topic_in_;
};

}  // namespace webrtc_bridge

#endif  // WEBRTC_BRIDGE__DATA_CHANNEL_HANDLER_HPP_
