/**
 * @file signaling_server.cpp
 * @brief Implements WebRTC signaling, peer management, and media channel setup.
 * @author Mark West (mark@telemething.com)
 */

#include "webrtc_bridge/signaling_server.hpp"
#include <nlohmann/json.hpp>
#include <iostream>
#include <sstream>

using json = nlohmann::json;

namespace webrtc_bridge
{

SignalingServer::SignalingServer(uint16_t port)
  : port_(port)
{
  rtc_config_.iceServers.emplace_back("stun:stun.l.google.com:19302");
}

SignalingServer::~SignalingServer()
{
  stop();
}

void SignalingServer::start()
{
  rtc::WebSocketServer::Configuration ws_config;
  ws_config.port = port_;
  ws_config.enableTls = false;

  ws_server_ = std::make_unique<rtc::WebSocketServer>(ws_config);

  ws_server_->onClient([this](std::shared_ptr<rtc::WebSocket> ws) {
    std::string peer_id = generate_peer_id();

    std::cout << "[SignalingServer] New client connected: " << peer_id << std::endl;

    ws->onOpen([this, peer_id, ws]() {
      std::cout << "[SignalingServer] WebSocket opened for peer: " << peer_id << std::endl;
      setup_peer_connection(peer_id, ws);
    });

    ws->onMessage([this, peer_id](rtc::message_variant data) {
      if (std::holds_alternative<std::string>(data)) {
        handle_websocket_message(peer_id, std::get<std::string>(data));
      }
    });

    ws->onClosed([this, peer_id]() {
      std::cout << "[SignalingServer] WebSocket closed for peer: " << peer_id << std::endl;
      {
        std::lock_guard<std::mutex> lock(peers_mutex_);
        peers_.erase(peer_id);
      }
      if (on_peer_disconnected_) {
        on_peer_disconnected_(peer_id);
      }
    });

    ws->onError([peer_id](const std::string& error) {
      std::cerr << "[SignalingServer] WebSocket error for peer " << peer_id << ": " << error << std::endl;
    });
  });

  std::cout << "[SignalingServer] Started on port " << port_ << std::endl;
}

void SignalingServer::stop()
{
  if (ws_server_) {
    ws_server_->stop();
    ws_server_.reset();
  }

  std::lock_guard<std::mutex> lock(peers_mutex_);
  peers_.clear();
}

void SignalingServer::setup_peer_connection(const std::string& peer_id, std::shared_ptr<rtc::WebSocket> ws)
{
  auto pc = std::make_shared<rtc::PeerConnection>(rtc_config_);

  PeerContext context;
  context.id = peer_id;
  context.peer_connection = pc;
  context.websocket = ws;

  pc->onStateChange([peer_id](rtc::PeerConnection::State state) {
    std::cout << "[SignalingServer] Peer " << peer_id << " state: " << static_cast<int>(state) << std::endl;
  });

  pc->onGatheringStateChange([peer_id, ws](rtc::PeerConnection::GatheringState state) {
    std::cout << "[SignalingServer] Peer " << peer_id << " gathering state: " << static_cast<int>(state) << std::endl;
  });

  pc->onLocalDescription([ws](rtc::Description desc) {
    json msg;
    msg["type"] = desc.typeString();
    msg["sdp"] = std::string(desc);

    if (ws->isOpen()) {
      ws->send(msg.dump());
    }
  });

  pc->onLocalCandidate([ws](rtc::Candidate candidate) {
    json msg;
    msg["type"] = "candidate";
    msg["candidate"] = std::string(candidate);
    msg["mid"] = candidate.mid();

    if (ws->isOpen()) {
      ws->send(msg.dump());
    }
  });

  pc->onDataChannel([this, peer_id](std::shared_ptr<rtc::DataChannel> dc) {
    std::cout << "[SignalingServer] Data channel received: " << dc->label() << std::endl;

    dc->onMessage([this, peer_id, label = dc->label()](rtc::message_variant data) {
      if (std::holds_alternative<std::string>(data)) {
        if (on_data_channel_message_) {
          on_data_channel_message_(peer_id, label, std::get<std::string>(data));
        }
      }
    });

    std::lock_guard<std::mutex> lock(peers_mutex_);
    auto it = peers_.find(peer_id);
    if (it != peers_.end()) {
      it->second.data_channels.push_back(dc);
    }
  });

  create_data_channels(context);
  create_video_track(context);

  {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    peers_[peer_id] = std::move(context);
  }

  if (on_peer_connected_) {
    std::lock_guard<std::mutex> lock(peers_mutex_);
    on_peer_connected_(peer_id, peers_[peer_id]);
  }

  pc->setLocalDescription();
}

void SignalingServer::create_data_channels(PeerContext& context)
{
  auto dc_out = context.peer_connection->createDataChannel("ros_topic_out");
  dc_out->onOpen([label = dc_out->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });
  context.data_channels.push_back(dc_out);

  auto dc_in = context.peer_connection->createDataChannel("ros_topic_in");
  dc_in->onOpen([label = dc_in->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_in->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<std::string>(data)) {
      if (on_data_channel_message_) {
        on_data_channel_message_(peer_id, "ros_topic_in", std::get<std::string>(data));
      }
    }
  });

  context.data_channels.push_back(dc_in);

  // Video data channel for JPEG frames
  rtc::DataChannelInit video_config;
  video_config.reliability.unordered = true;
  video_config.reliability.maxRetransmits = 0;

  auto dc_video = context.peer_connection->createDataChannel("video_frames", video_config);
  dc_video->onOpen([label = dc_video->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });
  context.data_channels.push_back(dc_video);

  // Steering data channel for receiving int16 steering values
  auto dc_steering = context.peer_connection->createDataChannel("steering");
  dc_steering->onOpen([label = dc_steering->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_steering->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "steering", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_steering);

  // Throttle data channel for receiving int16 throttle values
  auto dc_throttle = context.peer_connection->createDataChannel("throttle");
  dc_throttle->onOpen([label = dc_throttle->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_throttle->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "throttle", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_throttle);

  // Brake data channel for receiving int16 brake values
  auto dc_brake = context.peer_connection->createDataChannel("brake");
  dc_brake->onOpen([label = dc_brake->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_brake->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "brake", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_brake);

  // Gear data channel for receiving int16 gear values
  auto dc_gear = context.peer_connection->createDataChannel("gear");
  dc_gear->onOpen([label = dc_gear->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_gear->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "gear", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_gear);

  // Start data channel for receiving int16 start button values (momentary switch)
  auto dc_start = context.peer_connection->createDataChannel("start");
  dc_start->onOpen([label = dc_start->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_start->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "start", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_start);

  // Panic data channel for receiving int16 panic button values (momentary switch)
  auto dc_panic = context.peer_connection->createDataChannel("panic");
  dc_panic->onOpen([label = dc_panic->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_panic->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "panic", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_panic);

  // Mode data channel for receiving int16 mode values (0=IDLE, 1=CONFIG, 2=RUN, 3=MANUAL)
  auto dc_mode = context.peer_connection->createDataChannel("mode");
  dc_mode->onOpen([label = dc_mode->label()]() {
    std::cout << "[SignalingServer] Data channel opened: " << label << std::endl;
  });

  dc_mode->onMessage([this, peer_id = context.id](rtc::message_variant data) {
    if (std::holds_alternative<rtc::binary>(data)) {
      const auto& binary = std::get<rtc::binary>(data);
      if (on_binary_message_ && binary.size() >= 2) {
        on_binary_message_(peer_id, "mode", binary.data(), binary.size());
      }
    }
  });

  context.data_channels.push_back(dc_mode);
}

void SignalingServer::create_video_track(PeerContext& context)
{
  const rtc::SSRC ssrc = 42;
  const int payloadType = 96;
  const uint32_t clockRate = 90000;

  rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
  media.addH264Codec(payloadType);
  media.addSSRC(ssrc, "video-stream");

  auto track = context.peer_connection->addTrack(media);

  auto rtp_config = std::make_shared<rtc::RtpPacketizationConfig>(
    ssrc, "video-stream", payloadType, clockRate
  );

  auto packetizer = std::make_shared<rtc::H264RtpPacketizer>(
    rtc::H264RtpPacketizer::Separator::LongStartSequence, rtp_config
  );

  auto srReporter = std::make_shared<rtc::RtcpSrReporter>(rtp_config);
  packetizer->addToChain(srReporter);

  auto nackResponder = std::make_shared<rtc::RtcpNackResponder>();
  srReporter->addToChain(nackResponder);

  track->setMediaHandler(packetizer);

  context.video_track = track;

  track->onOpen([](){
    std::cout << "[SignalingServer] Video track opened" << std::endl;
  });
}

void SignalingServer::handle_websocket_message(const std::string& peer_id, const std::string& message)
{
  try {
    json msg = json::parse(message);
    std::string type = msg["type"].get<std::string>();

    std::lock_guard<std::mutex> lock(peers_mutex_);
    auto it = peers_.find(peer_id);
    if (it == peers_.end()) {
      return;
    }

    auto& context = it->second;

    if (type == "answer") {
      std::string sdp = msg["sdp"].get<std::string>();
      context.peer_connection->setRemoteDescription(rtc::Description(sdp, type));
    } else if (type == "candidate") {
      std::string candidate = msg["candidate"].get<std::string>();
      std::string mid = msg["mid"].get<std::string>();
      context.peer_connection->addRemoteCandidate(rtc::Candidate(candidate, mid));
    }
  } catch (const std::exception& e) {
    std::cerr << "[SignalingServer] Error parsing message: " << e.what() << std::endl;
  }
}

void SignalingServer::set_on_peer_connected(OnPeerConnectedCallback callback)
{
  on_peer_connected_ = std::move(callback);
}

void SignalingServer::set_on_peer_disconnected(OnPeerDisconnectedCallback callback)
{
  on_peer_disconnected_ = std::move(callback);
}

void SignalingServer::set_on_data_channel_message(OnDataChannelMessageCallback callback)
{
  on_data_channel_message_ = std::move(callback);
}

void SignalingServer::set_on_binary_message(OnBinaryMessageCallback callback)
{
  on_binary_message_ = std::move(callback);
}

void SignalingServer::send_to_peer(const std::string& peer_id, const std::string& channel_label,
                                    const std::string& message)
{
  std::lock_guard<std::mutex> lock(peers_mutex_);
  auto it = peers_.find(peer_id);
  if (it == peers_.end()) {
    return;
  }

  for (auto& dc : it->second.data_channels) {
    if (dc->label() == channel_label && dc->isOpen()) {
      dc->send(message);
      break;
    }
  }
}

void SignalingServer::send_video_frame(const std::string& peer_id, const uint8_t* data,
                                        size_t size, uint32_t timestamp)
{
  (void)timestamp;

  std::lock_guard<std::mutex> lock(peers_mutex_);
  auto it = peers_.find(peer_id);
  if (it == peers_.end()) {
    return;
  }

  // Send via video_frames data channel as binary
  for (auto& dc : it->second.data_channels) {
    if (dc->label() == "video_frames" && dc->isOpen()) {
      dc->send(reinterpret_cast<const std::byte*>(data), size);
      break;
    }
  }
}

void SignalingServer::broadcast_to_data_channel(const std::string& channel_label,
                                                 const std::string& message)
{
  std::lock_guard<std::mutex> lock(peers_mutex_);
  for (auto& [peer_id, context] : peers_) {
    for (auto& dc : context.data_channels) {
      if (dc->label() == channel_label && dc->isOpen()) {
        dc->send(message);
      }
    }
  }
}

std::vector<std::string> SignalingServer::get_connected_peers() const
{
  std::lock_guard<std::mutex> lock(peers_mutex_);
  std::vector<std::string> peer_ids;
  for (const auto& [id, _] : peers_) {
    peer_ids.push_back(id);
  }
  return peer_ids;
}

std::string SignalingServer::generate_peer_id()
{
  std::ostringstream oss;
  oss << "peer_" << (++peer_counter_);
  return oss.str();
}

}  // namespace webrtc_bridge
