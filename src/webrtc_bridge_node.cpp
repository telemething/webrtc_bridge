/**
 * @file webrtc_bridge_node.cpp
 * @brief Implements the WebRTC bridge ROS2 node behavior.
 * @author Mark West (mark@telemething.com)
 */

#include "webrtc_bridge/webrtc_bridge_node.hpp"
#include <chrono>

namespace webrtc_bridge
{

WebRTCBridgeNode::WebRTCBridgeNode(const rclcpp::NodeOptions& options)
  : rclcpp::Node("webrtc_bridge_node", options)
{
  declare_parameters();
  initialize_components();

  RCLCPP_INFO(get_logger(), "WebRTC Bridge Node started");
}

WebRTCBridgeNode::~WebRTCBridgeNode()
{
  if (signaling_server_) {
    signaling_server_->stop();
  }
  if (video_encoder_) {
    video_encoder_->shutdown();
  }

  RCLCPP_INFO(get_logger(), "WebRTC Bridge Node shutdown");
}

void WebRTCBridgeNode::declare_parameters()
{
  signaling_port_ = declare_parameter("signaling_port", 8080);
  video_topic_ = declare_parameter("video_topic", "/camera/image_raw");
  string_topic_out_ = declare_parameter("string_topic_out", "/webrtc/string_out");
  string_topic_in_ = declare_parameter("string_topic_in", "/webrtc/string_in");
  steering_topic_ = declare_parameter("steering_topic", "/steering");
  video_width_ = declare_parameter("video_width", 640);
  video_height_ = declare_parameter("video_height", 480);
  video_fps_ = declare_parameter("video_fps", 30);
  video_bitrate_ = declare_parameter("video_bitrate", 1000);

  RCLCPP_INFO(get_logger(), "Parameters:");
  RCLCPP_INFO(get_logger(), "  signaling_port: %d", signaling_port_);
  RCLCPP_INFO(get_logger(), "  video_topic: %s", video_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  string_topic_out: %s", string_topic_out_.c_str());
  RCLCPP_INFO(get_logger(), "  string_topic_in: %s", string_topic_in_.c_str());
  RCLCPP_INFO(get_logger(), "  steering_topic: %s", steering_topic_.c_str());
  RCLCPP_INFO(get_logger(), "  video: %dx%d @ %d fps, %d kbps",
              video_width_, video_height_, video_fps_, video_bitrate_);
}

void WebRTCBridgeNode::initialize_components()
{
  signaling_server_ = std::make_unique<SignalingServer>(signaling_port_);

  signaling_server_->set_on_peer_connected(
    [this](const std::string& peer_id, SignalingServer::PeerContext& context) {
      on_peer_connected(peer_id, context);
    }
  );

  signaling_server_->set_on_peer_disconnected(
    [this](const std::string& peer_id) {
      on_peer_disconnected(peer_id);
    }
  );

  signaling_server_->set_on_data_channel_message(
    [this](const std::string& peer_id, const std::string& channel_label,
           const std::string& message) {
      on_data_channel_message(peer_id, channel_label, message);
    }
  );

  signaling_server_->set_on_binary_message(
    [this](const std::string& peer_id, const std::string& channel_label,
           const std::byte* data, size_t size) {
      on_binary_message(peer_id, channel_label, data, size);
    }
  );

  signaling_server_->start();

  VideoEncoder::Config encoder_config;
  encoder_config.width = video_width_;
  encoder_config.height = video_height_;
  encoder_config.fps = video_fps_;
  encoder_config.bitrate_kbps = video_bitrate_;

  video_encoder_ = std::make_unique<VideoEncoder>(encoder_config);
  video_encoder_->initialize();

  video_encoder_->set_encoded_frame_callback(
    [this](const uint8_t* data, size_t size, uint32_t timestamp, bool is_keyframe) {
      on_encoded_frame(data, size, timestamp, is_keyframe);
    }
  );

  auto node_ptr = std::shared_ptr<rclcpp::Node>(this, [](rclcpp::Node*){});
  data_channel_handler_ = std::make_unique<DataChannelHandler>(
    node_ptr, string_topic_out_, string_topic_in_
  );

  data_channel_handler_->set_send_callback(
    [this](const std::string& channel_label, const std::string& message) {
      signaling_server_->broadcast_to_data_channel(channel_label, message);
    }
  );

  image_subscription_ = create_subscription<sensor_msgs::msg::Image>(
    video_topic_, 10,
    std::bind(&WebRTCBridgeNode::image_callback, this, std::placeholders::_1)
  );

  steering_publisher_ = create_publisher<std_msgs::msg::Int16>(steering_topic_, 10);

  RCLCPP_INFO(get_logger(), "Subscribed to image topic: %s", video_topic_.c_str());
  RCLCPP_INFO(get_logger(), "Publishing steering to: %s", steering_topic_.c_str());
}

void WebRTCBridgeNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
  try {
    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");

    uint64_t timestamp_ns = msg->header.stamp.sec * 1000000000ULL + msg->header.stamp.nanosec;

    video_encoder_->encode_frame(cv_ptr->image, timestamp_ns);
  } catch (const cv_bridge::Exception& e) {
    RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
  }
}

void WebRTCBridgeNode::on_peer_connected(const std::string& peer_id,
                                          SignalingServer::PeerContext& context)
{
  (void)context;
  RCLCPP_INFO(get_logger(), "Peer connected: %s", peer_id.c_str());

  video_encoder_->request_keyframe();
}

void WebRTCBridgeNode::on_peer_disconnected(const std::string& peer_id)
{
  RCLCPP_INFO(get_logger(), "Peer disconnected: %s", peer_id.c_str());
}

void WebRTCBridgeNode::on_data_channel_message(const std::string& peer_id,
                                                const std::string& channel_label,
                                                const std::string& message)
{
  RCLCPP_DEBUG(get_logger(), "Message from %s on channel '%s': %s",
               peer_id.c_str(), channel_label.c_str(), message.c_str());

  data_channel_handler_->on_webrtc_message(channel_label, message);
}

void WebRTCBridgeNode::on_binary_message(const std::string& peer_id,
                                          const std::string& channel_label,
                                          const std::byte* data, size_t size)
{
  if (channel_label == "steering" && size >= 2) {
    // Interpret as little-endian int16
    int16_t steering_value = static_cast<int16_t>(
      static_cast<uint8_t>(data[0]) |
      (static_cast<uint8_t>(data[1]) << 8)
    );

    auto msg = std_msgs::msg::Int16();
    msg.data = steering_value;
    steering_publisher_->publish(msg);

    RCLCPP_DEBUG(get_logger(), "Steering from %s: %d", peer_id.c_str(), steering_value);
  }
}

void WebRTCBridgeNode::on_encoded_frame(const uint8_t* data, size_t size,
                                         uint32_t timestamp, bool is_keyframe)
{
  (void)is_keyframe;

  for (const auto& peer_id : signaling_server_->get_connected_peers()) {
    signaling_server_->send_video_frame(peer_id, data, size, timestamp);
  }
}

}  // namespace webrtc_bridge
