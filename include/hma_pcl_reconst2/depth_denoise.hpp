#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

namespace hma_pcl_reconst2
{

// Depth-image denoiser for active-stereo depth sensors (RealSense, Orbbec Gemini).
//
// ToF / structured-light sensors such as the Xtion do not need this; keep it disabled
// (the default) and the depth image is passed through untouched.
//
// Active stereo triangulates z = f*b/d, so the depth error grows with the square of
// the range:
//
//   sigma_z(z) = z^2 * sigma_d / (f * b)
//
// For a Gemini 336L (b ~= 0.095 m, f ~= 640 px, sigma_d ~= 0.15 px) that is about
// 2.5 mm at 1 m but 25 cm at 10 m. Every threshold in this class therefore has the
// form
//
//   tol(z) = base + quad * z^2
//
// where the quadratic coefficient is the standard deviation expressed in inverse
// depth (1/m), i.e. the filter behaves like a constant-sigma filter in disparity
// space. A fixed threshold cannot work over a 0.3-10 m range: tuned for the near
// field it shreds far surfaces, tuned for the far field it destroys near geometry.
class DepthDenoiser
{
public:
  struct Params
  {
    bool enable = false;
    int num_threads = 0;            // 0: leave OpenCV's global thread count alone

    // 1. range clip
    bool clip_enable = true;
    float min_depth = 0.3f;         // [m]
    float max_depth = 10.0f;        // [m]

    // 2. speckle removal (depth-dependent connectivity tolerance)
    bool speckle_enable = true;
    int speckle_max_size = 200;     // [px] components smaller than this are dropped
    // Must clear the *adjacent-pixel* difference, whose sigma is sqrt(2)*sigma_z,
    // with margin: tol = 3 * sqrt(2) * sigma_z ~= 4.3 * sigma_z. Set too tight, the
    // flood fill fragments a noisy far surface and the whole surface is deleted.
    float speckle_diff_base = 0.005f;  // [m]
    float speckle_diff_quad = 0.011f;  // [1/m]

    // 3. NaN-aware bilateral (depth-dependent range sigma)
    bool bilateral_enable = true;
    int bilateral_radius = 2;          // 0 disables; 2 => 5x5
    float bilateral_sigma_space = 1.5f;   // [px]
    float bilateral_sigma_base = 0.005f;  // [m]
    float bilateral_sigma_quad = 0.004f;  // [1/m] == sigma in inverse-depth space
    int bilateral_min_valid = 4;          // taps required before a pixel is smoothed
    bool bilateral_invalidate_sparse = false;

    // 4. temporal EMA (depth-dependent motion gate)
    bool temporal_enable = true;
    // Weight of the new frame; 0 disables. This is the strongest lever at long
    // range: at 10 m it takes the plane RMS from ~250 mm to ~59 mm, well past what
    // the spatial filter alone reaches, because stereo noise is nearly uncorrelated
    // between frames.
    float temporal_alpha = 0.5f;
    float temporal_diff_base = 0.02f;  // [m]
    float temporal_diff_quad = 0.008f; // [1/m]
    double temporal_max_gap_sec = 0.5; // reset the EMA after a gap this long
  };

  DepthDenoiser();

  // Declares every denoise.* parameter on the node. Call once, from the constructor.
  static void declareParameters(rclcpp::Node * node);

  // Reads the declared parameters and builds the lookup tables.
  void configure(rclcpp::Node * node);

  // Same, from an explicit parameter set. Lets the filter be exercised without a node.
  void configure(const Params & params);

  bool enabled() const {return params_.enable;}
  const Params & params() const {return params_;}

  // One-line summary of the active configuration, for logging.
  std::string describe() const;

  // Drops the temporal state. Call on unsubscribe or whenever the stream restarts.
  void reset();

  // Filters `in` into a freshly allocated `out` with the same encoding, size and
  // header. Returns false when nothing was done (disabled, unsupported encoding or
  // a malformed message); `out` is then left untouched and the caller must keep
  // using `in`.
  bool apply(
    const sensor_msgs::msg::Image::ConstSharedPtr & in,
    sensor_msgs::msg::Image::SharedPtr & out);

private:
  void buildTables();

  void decode(const sensor_msgs::msg::Image & in, bool is_16u);
  void encode(sensor_msgs::msg::Image & out, bool is_16u) const;

  void applyClip();
  void applySpeckle();
  void applyBilateral();
  void applyTemporal(int64_t stamp_ns, const std::string & frame_id);

  Params params_;
  bool configured_ = false;

  rclcpp::Logger logger_;
  rclcpp::Clock::SharedPtr clock_;

  cv::Mat work_;   // CV_32FC1, metres, NaN marks an invalid pixel
  cv::Mat tmp_;    // CV_32FC1, bilateral destination
  cv::Mat prev_;   // CV_32FC1, temporal EMA state

  // Scratch for the speckle stage, kept across frames to avoid reallocating.
  // Connected components are built from horizontal runs joined by union-find
  // rather than by a per-pixel flood fill: same result, but every pass walks
  // memory in row order, which at 1280x720 is the difference between ~49 ms and
  // a few ms per frame.
  std::vector<int32_t> run_row_;
  std::vector<int32_t> run_x0_;
  std::vector<int32_t> run_x1_;
  std::vector<int32_t> uf_parent_;
  std::vector<int32_t> uf_size_;
  std::vector<int32_t> row_run_begin_;

  std::vector<float> spatial_w_;  // (2r+1)^2 Gaussian weights
  std::vector<float> range_lut_;  // exp(-t) sampled over t in [0, kRangeCutoff)
  float lut_scale_ = 0.0f;

  int64_t prev_stamp_ns_ = 0;
  std::string prev_frame_id_;
  bool has_prev_ = false;
};

}  // namespace hma_pcl_reconst2
