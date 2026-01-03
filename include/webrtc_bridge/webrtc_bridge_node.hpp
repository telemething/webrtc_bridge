#ifndef WEBRTC_BRIDGE__WEBRTC_BRIDGE_NODE_HPP_
#define WEBRTC_BRIDGE__WEBRTC_BRIDGE_NODE_HPP_

/**
 * @file webrtc_bridge_node.hpp
 * @brief Defines the ROS2 node that bridges WebRTC signaling, media, and ROS topics.
 * @author Mark West (mark@telemething.com)
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int16.hpp>
#include <cv_bridge/cv_bridge.h>

#include "webrtc_bridge/signaling_server.hpp"
#include "webrtc_bridge/video_encoder.hpp"
#include "webrtc_bridge/data_channel_handler.hpp"

#include <memory>
#include <string>

namespace webrtc_bridge
{

/**
 * @brief ROS2 node that wires image input, WebRTC signaling, and data channels together.
 */
class WebRTCBridgeNode : public rclcpp::Node
{
public:
  /**
   * @brief Construct the bridge node with optional ROS2 node options.
   * @param options ROS2 node options for composition contexts.
   */
  explicit WebRTCBridgeNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  /**
   * @brief Destructor ensures signaling and encoding components shut down cleanly.
   */
  ~WebRTCBridgeNode();

private:
  /**
   * @brief Declare ROS2 parameters controlling signaling and video streaming behavior.
   */
  void declare_parameters();

  /**
   * @brief Create signaling, encoder, data channel, and subscription components.
   */
  void initialize_components();

  /**
   * @brief Handle incoming images and forward them to the encoder.
   * @param msg Incoming ROS image message.
   */
  void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);

  /**
   * @brief Notify components when a peer connects and request an initial keyframe.
   * @param peer_id Identifier assigned to the connected peer.
   * @param context Peer context containing connection resources.
   */
  void on_peer_connected(const std::string& peer_id, SignalingServer::PeerContext& context);

  /**
   * @brief Handle peer disconnection cleanup.
   * @param peer_id Identifier of the disconnected peer.
   */
  void on_peer_disconnected(const std::string& peer_id);

  /**
   * @brief Dispatch inbound WebRTC data channel messages to the ROS bridge logic.
   * @param peer_id Sender peer identifier.
   * @param channel_label Data channel label.
   * @param message UTF-8 payload received.
   */
  void on_data_channel_message(const std::string& peer_id,
                               const std::string& channel_label,
                               const std::string& message);

  /**
   * @brief Handle binary data channel messages (e.g., steering input).
   * @param peer_id Sender peer identifier.
   * @param channel_label Data channel label.
   * @param data Pointer to binary data.
   * @param size Size of the binary data.
   */
  void on_binary_message(const std::string& peer_id,
                         const std::string& channel_label,
                         const std::byte* data, size_t size);

  /**
   * @brief Relay encoded frames to all connected peers via the signaling server.
   * @param data Encoded frame bytes (JPEG).
   * @param size Size of the encoded buffer.
   * @param timestamp RTP timestamp for the frame.
   * @param is_keyframe True when the frame starts a new GOP.
   */
  void on_encoded_frame(const uint8_t* data, size_t size,
                        uint32_t timestamp, bool is_keyframe);

  // Parameters
  int signaling_port_;
  std::string video_topic_;
  std::string string_topic_out_;
  std::string string_topic_in_;
  std::string steering_topic_;
  std::string throttle_topic_;
  std::string brake_topic_;
  std::string gear_topic_;
  std::string start_topic_;
  std::string panic_topic_;
  std::string mode_topic_;
  int video_width_;
  int video_height_;
  int video_fps_;
  int video_bitrate_;

  // Components
  std::unique_ptr<SignalingServer> signaling_server_;
  std::unique_ptr<VideoEncoder> video_encoder_;
  std::unique_ptr<DataChannelHandler> data_channel_handler_;

  // ROS2 interfaces
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr steering_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr throttle_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr brake_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr gear_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr start_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr panic_publisher_;
  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr mode_publisher_;
};

}  // namespace webrtc_bridge

#endif  // WEBRTC_BRIDGE__WEBRTC_BRIDGE_NODE_HPP_
