/**
 * @file video_encoder.cpp
 * @brief Implements JPEG-based video encoding for WebRTC transport.
 * @author Mark West (mark@telemething.com)
 */

#include "webrtc_bridge/video_encoder.hpp"
#include <iostream>
#include <chrono>

namespace webrtc_bridge
{

VideoEncoder::VideoEncoder(const Config& config)
  : config_(config)
{
}

VideoEncoder::~VideoEncoder()
{
  shutdown();
}

bool VideoEncoder::initialize()
{
  if (initialized_) {
    return true;
  }

  start_timestamp_ns_ = 0;
  frame_count_ = 0;
  request_keyframe_ = true;
  initialized_ = true;

  std::cout << "[VideoEncoder] Initialized with " << config_.width << "x" << config_.height
            << " @ " << config_.fps << " fps, " << config_.bitrate_kbps << " kbps" << std::endl;

  return true;
}

void VideoEncoder::shutdown()
{
  if (!initialized_) {
    return;
  }

  initialized_ = false;
  std::cout << "[VideoEncoder] Shutdown" << std::endl;
}

void VideoEncoder::encode_frame(const cv::Mat& frame, uint64_t timestamp_ns)
{
  if (!initialized_) {
    return;
  }

  if (start_timestamp_ns_ == 0) {
    start_timestamp_ns_ = timestamp_ns;
  }

  cv::Mat resized;
  if (frame.cols != config_.width || frame.rows != config_.height) {
    cv::resize(frame, resized, cv::Size(config_.width, config_.height));
  } else {
    resized = frame;
  }

  cv::Mat bgr;
  if (resized.channels() == 1) {
    cv::cvtColor(resized, bgr, cv::COLOR_GRAY2BGR);
  } else if (resized.channels() == 4) {
    cv::cvtColor(resized, bgr, cv::COLOR_BGRA2BGR);
  } else {
    bgr = resized;
  }

  encode_as_jpeg(bgr, timestamp_ns);
  frame_count_++;
}

void VideoEncoder::encode_as_jpeg(const cv::Mat& frame, uint64_t timestamp_ns)
{
  bool is_keyframe = request_keyframe_.exchange(false) ||
                     (frame_count_ % config_.keyframe_interval == 0);

  std::vector<uchar> buffer;
  std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
  cv::imencode(".jpg", frame, buffer, params);

  uint32_t rtp_timestamp = timestamp_to_rtp(timestamp_ns);

  std::lock_guard<std::mutex> lock(callback_mutex_);
  if (encoded_frame_callback_) {
    encoded_frame_callback_(buffer.data(), buffer.size(), rtp_timestamp, is_keyframe);
  }
}

uint32_t VideoEncoder::timestamp_to_rtp(uint64_t timestamp_ns)
{
  uint64_t elapsed_ns = timestamp_ns - start_timestamp_ns_;
  return static_cast<uint32_t>((elapsed_ns * 90) / 1000000);
}

void VideoEncoder::request_keyframe()
{
  request_keyframe_ = true;
}

void VideoEncoder::set_encoded_frame_callback(EncodedFrameCallback callback)
{
  std::lock_guard<std::mutex> lock(callback_mutex_);
  encoded_frame_callback_ = std::move(callback);
}

void VideoEncoder::set_bitrate(int bitrate_kbps)
{
  config_.bitrate_kbps = bitrate_kbps;
}

}  // namespace webrtc_bridge
