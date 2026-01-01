#ifndef WEBRTC_BRIDGE__SIGNALING_SERVER_HPP_
#define WEBRTC_BRIDGE__SIGNALING_SERVER_HPP_

/**
 * @file signaling_server.hpp
 * @brief Declares the WebRTC signaling and transport helper for peer management.
 * @author Mark West (mark@telemething.com)
 */

#include <rtc/rtc.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

namespace webrtc_bridge
{

/**
 * @brief Hosts WebSocket signaling and manages WebRTC peer connections, tracks, and data channels.
 */
class SignalingServer
{
public:
  using PeerConnectionPtr = std::shared_ptr<rtc::PeerConnection>;
  using DataChannelPtr = std::shared_ptr<rtc::DataChannel>;
  using TrackPtr = std::shared_ptr<rtc::Track>;

  /**
   * @brief Aggregates the resources associated with a single connected peer.
   */
  struct PeerContext
  {
    std::string id;                        ///< Stable identifier assigned to the peer.
    PeerConnectionPtr peer_connection;     ///< Underlying WebRTC peer connection.
    std::shared_ptr<rtc::WebSocket> websocket;  ///< WebSocket used for signaling.
    std::vector<DataChannelPtr> data_channels;  ///< Data channels bound to the peer.
    TrackPtr video_track;                  ///< Media track for video transport.
  };

  using OnPeerConnectedCallback = std::function<void(const std::string& peer_id, PeerContext& context)>;
  using OnPeerDisconnectedCallback = std::function<void(const std::string& peer_id)>;
  using OnDataChannelMessageCallback = std::function<void(const std::string& peer_id,
                                                          const std::string& channel_label,
                                                          const std::string& message)>;

  /**
   * @brief Construct a signaling server bound to a TCP port.
   * @param port TCP port for the WebSocket signaling server.
   */
  SignalingServer(uint16_t port);

  /**
   * @brief Destructor stops the server and clears peer state.
   */
  ~SignalingServer();

  /**
   * @brief Start listening for WebSocket clients and configure callbacks.
   */
  void start();

  /**
   * @brief Stop the signaling server and drop active peer connections.
   */
  void stop();

  /**
   * @brief Register a callback invoked when a peer connects.
   * @param callback Function receiving peer id and context.
   */
  void set_on_peer_connected(OnPeerConnectedCallback callback);

  /**
   * @brief Register a callback invoked when a peer disconnects.
   * @param callback Function receiving peer id.
   */
  void set_on_peer_disconnected(OnPeerDisconnectedCallback callback);

  /**
   * @brief Register a callback invoked when data channel messages arrive.
   * @param callback Function receiving peer id, channel label, and payload.
   */
  void set_on_data_channel_message(OnDataChannelMessageCallback callback);

  /**
   * @brief Send a UTF-8 message to a specific peer's data channel.
   * @param peer_id Target peer identifier.
   * @param channel_label Data channel label to send on.
   * @param message Payload to send.
   */
  void send_to_peer(const std::string& peer_id, const std::string& channel_label,
                    const std::string& message);

  /**
   * @brief Send encoded video data to a specific peer's video channel.
   * @param peer_id Target peer identifier.
   * @param data Encoded frame buffer.
   * @param size Buffer size in bytes.
   * @param timestamp RTP timestamp associated with the frame.
   */
  void send_video_frame(const std::string& peer_id, const uint8_t* data, size_t size,
                        uint32_t timestamp);

  /**
   * @brief Broadcast a message across all peers on the given channel label.
   * @param channel_label Channel label to match.
   * @param message Payload to send.
   */
  void broadcast_to_data_channel(const std::string& channel_label, const std::string& message);

  /**
   * @brief Get a snapshot of currently connected peer identifiers.
   * @return Vector of peer ids.
   */
  std::vector<std::string> get_connected_peers() const;

private:
  /**
   * @brief Parse and handle incoming WebSocket signaling messages.
   * @param peer_id Sender peer identifier.
   * @param message Raw JSON signaling message.
   */
  void handle_websocket_message(const std::string& peer_id, const std::string& message);

  /**
   * @brief Create the WebRTC peer connection and hook up callbacks for a new peer.
   * @param peer_id Assigned peer identifier.
   * @param ws WebSocket transport for signaling with the peer.
   */
  void setup_peer_connection(const std::string& peer_id, std::shared_ptr<rtc::WebSocket> ws);

  /**
   * @brief Create data channels for ROS topic ingress/egress and video transport.
   * @param context Peer context to populate.
   */
  void create_data_channels(PeerContext& context);

  /**
   * @brief Create and configure the outbound video track for the peer.
   * @param context Peer context to populate.
   */
  void create_video_track(PeerContext& context);

  /**
   * @brief Generate a new unique peer identifier.
   * @return Generated identifier.
   */
  std::string generate_peer_id();

  uint16_t port_;
  std::unique_ptr<rtc::WebSocketServer> ws_server_;
  std::unordered_map<std::string, PeerContext> peers_;
  mutable std::mutex peers_mutex_;

  OnPeerConnectedCallback on_peer_connected_;
  OnPeerDisconnectedCallback on_peer_disconnected_;
  OnDataChannelMessageCallback on_data_channel_message_;

  rtc::Configuration rtc_config_;
  uint64_t peer_counter_ = 0;
};

}  // namespace webrtc_bridge

#endif  // WEBRTC_BRIDGE__SIGNALING_SERVER_HPP_
