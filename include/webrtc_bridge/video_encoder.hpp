#ifndef WEBRTC_BRIDGE__VIDEO_ENCODER_HPP_
#define WEBRTC_BRIDGE__VIDEO_ENCODER_HPP_

/**
 * @file video_encoder.hpp
 * @brief Declares a lightweight JPEG-based video encoder for WebRTC transport.
 * @author Mark West (mark@telemething.com)
 */

#include <opencv2/opencv.hpp>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <atomic>

namespace webrtc_bridge
{

/**
 * @brief Encodes OpenCV frames to JPEG and emits RTP-compatible timestamps.
 */
class VideoEncoder
{
public:
  using EncodedFrameCallback = std::function<void(const uint8_t* data, size_t size,
                                                   uint32_t timestamp, bool is_keyframe)>;

  /**
   * @brief Configuration parameters controlling encoding properties.
   */
  struct Config
  {
    int width;              ///< Output frame width in pixels.
    int height;             ///< Output frame height in pixels.
    int fps;                ///< Target frames per second.
    int bitrate_kbps;       ///< Estimated bitrate used for informational logging.
    int keyframe_interval;  ///< Interval in frames for forcing keyframes.

    Config() : width(640), height(480), fps(30), bitrate_kbps(1000), keyframe_interval(30) {}
  };

  /**
   * @brief Construct an encoder with optional configuration.
   * @param config Initial encoder settings.
   */
  explicit VideoEncoder(const Config& config = Config());

  /**
   * @brief Destructor shuts down encoder resources.
   */
  ~VideoEncoder();

  /**
   * @brief Initialize encoder state and timestamps.
   * @return True when initialization succeeds.
   */
  bool initialize();

  /**
   * @brief Stop encoding operations and reset state.
   */
  void shutdown();

  /**
   * @brief Encode a single OpenCV frame and emit it through the callback.
   * @param frame BGR image to encode.
   * @param timestamp_ns Monotonic timestamp in nanoseconds.
   */
  void encode_frame(const cv::Mat& frame, uint64_t timestamp_ns);

  /**
   * @brief Request that the next frame be treated as a keyframe.
   */
  void request_keyframe();

  /**
   * @brief Register a callback invoked when a frame finishes encoding.
   * @param callback Function receiving encoded buffer information.
   */
  void set_encoded_frame_callback(EncodedFrameCallback callback);

  /**
   * @brief Retrieve the current encoder configuration.
   * @return Active configuration copy.
   */
  Config get_config() const { return config_; }

  /**
   * @brief Update the nominal bitrate for informational purposes.
   * @param bitrate_kbps New bitrate in kilobits per second.
   */
  void set_bitrate(int bitrate_kbps);

private:
  /**
   * @brief Encode the provided frame to JPEG and emit it.
   * @param frame Preprocessed frame ready for encoding.
   * @param timestamp_ns Capture timestamp in nanoseconds.
   */
  void encode_as_jpeg(const cv::Mat& frame, uint64_t timestamp_ns);

  /**
   * @brief Convert a nanosecond timestamp to a 90 kHz RTP timestamp domain.
   * @param timestamp_ns Nanosecond timestamp.
   * @return RTP timestamp value.
   */
  uint32_t timestamp_to_rtp(uint64_t timestamp_ns);

  Config config_;
  EncodedFrameCallback encoded_frame_callback_;
  std::mutex callback_mutex_;

  std::atomic<bool> request_keyframe_{true};
  uint32_t frame_count_ = 0;
  uint64_t start_timestamp_ns_ = 0;
  bool initialized_ = false;
};

}  // namespace webrtc_bridge

#endif  // WEBRTC_BRIDGE__VIDEO_ENCODER_HPP_
