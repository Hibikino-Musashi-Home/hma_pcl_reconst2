#include "hma_pcl_reconst2/depth_denoise.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <sstream>
#include <utility>

#include <sensor_msgs/image_encodings.hpp>

namespace hma_pcl_reconst2
{

namespace
{

namespace enc = sensor_msgs::image_encodings;

constexpr float kNaN = std::numeric_limits<float>::quiet_NaN();

// The range weight is evaluated through a lookup table indexed by the normalised
// argument t = delta^2 / (2 * sigma^2). Normalising first means a single table
// serves every depth-dependent sigma. Taps beyond 3 sigma (t >= 4.5) are rejected.
constexpr float kRangeCutoff = 4.5f;
constexpr int kRangeLutSize = 512;

// Every threshold in this filter scales with the square of the range, because that
// is how stereo triangulation error behaves. `quad` is the sigma expressed in
// inverse depth (1/m).
inline float toleranceAt(float z, float base, float quad)
{
  return base + quad * z * z;
}

// DepthTraits<uint16_t>::fromMeters truncates instead of rounding (a systematic
// -0.5 mm bias on every point) and is undefined behaviour for NaN or for values
// beyond the uint16 range, both of which this filter produces by construction.
inline uint16_t metersToMillimeters(float z)
{
  if (!std::isfinite(z)) {
    return 0;
  }
  const long mm = std::lround(z * 1000.0f);
  if (mm <= 0) {
    return 0;
  }
  if (mm > 65535) {
    return 65535;
  }
  return static_cast<uint16_t>(mm);
}

inline int32_t ufFind(std::vector<int32_t> & parent, int32_t i)
{
  while (parent[static_cast<size_t>(i)] != i) {
    // Path halving: keeps find near-constant without a second pass.
    parent[static_cast<size_t>(i)] =
      parent[static_cast<size_t>(parent[static_cast<size_t>(i)])];
    i = parent[static_cast<size_t>(i)];
  }
  return i;
}

inline void ufUnion(
  std::vector<int32_t> & parent, std::vector<int32_t> & size, int32_t a, int32_t b)
{
  a = ufFind(parent, a);
  b = ufFind(parent, b);
  if (a == b) {
    return;
  }
  if (size[static_cast<size_t>(a)] < size[static_cast<size_t>(b)]) {
    std::swap(a, b);
  }
  parent[static_cast<size_t>(b)] = a;
  size[static_cast<size_t>(a)] += size[static_cast<size_t>(b)];
}

inline int64_t stampToNanoseconds(const builtin_interfaces::msg::Time & stamp)
{
  // Built from the raw header fields on purpose: rclcpp::Time defaults to
  // RCL_ROS_TIME and throws when compared against a differently-sourced clock.
  return static_cast<int64_t>(stamp.sec) * 1000000000LL +
         static_cast<int64_t>(stamp.nanosec);
}

}  // namespace

DepthDenoiser::DepthDenoiser()
: logger_(rclcpp::get_logger("depth_denoise"))
{
}

void DepthDenoiser::declareParameters(rclcpp::Node * node)
{
  const Params d;

  // Explicit template arguments throughout: the bare declare_parameter(name, 0)
  // form would declare an integer parameter and the later as_double() would throw.
  auto declare_bool = [node](const std::string & name, bool value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<bool>(name, value);
      }
    };
  auto declare_int = [node](const std::string & name, int value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<int>(name, value);
      }
    };
  auto declare_double = [node](const std::string & name, double value) {
      if (!node->has_parameter(name)) {
        node->declare_parameter<double>(name, value);
      }
    };

  // `denoise.enable`, not a bare `denoise` bool: a YAML key cannot be both a scalar
  // and a mapping, so a bare `denoise` alongside `denoise.*` would be impossible to
  // override from a parameter file. The launch argument is still called `denoise`.
  declare_bool("denoise.enable", d.enable);
  declare_int("denoise.num_threads", d.num_threads);

  declare_bool("denoise.clip.enable", d.clip_enable);
  declare_double("denoise.clip.min_depth", d.min_depth);
  declare_double("denoise.clip.max_depth", d.max_depth);

  declare_bool("denoise.speckle.enable", d.speckle_enable);
  declare_int("denoise.speckle.max_size", d.speckle_max_size);
  declare_double("denoise.speckle.diff_base", d.speckle_diff_base);
  declare_double("denoise.speckle.diff_quad", d.speckle_diff_quad);

  declare_bool("denoise.bilateral.enable", d.bilateral_enable);
  declare_int("denoise.bilateral.radius", d.bilateral_radius);
  declare_double("denoise.bilateral.sigma_space", d.bilateral_sigma_space);
  declare_double("denoise.bilateral.sigma_base", d.bilateral_sigma_base);
  declare_double("denoise.bilateral.sigma_quad", d.bilateral_sigma_quad);
  declare_int("denoise.bilateral.min_valid", d.bilateral_min_valid);
  declare_bool("denoise.bilateral.invalidate_sparse", d.bilateral_invalidate_sparse);

  declare_bool("denoise.temporal.enable", d.temporal_enable);
  declare_double("denoise.temporal.alpha", d.temporal_alpha);
  declare_double("denoise.temporal.diff_base", d.temporal_diff_base);
  declare_double("denoise.temporal.diff_quad", d.temporal_diff_quad);
  declare_double("denoise.temporal.max_gap_sec", d.temporal_max_gap_sec);
}

void DepthDenoiser::configure(rclcpp::Node * node)
{
  logger_ = node->get_logger().get_child("denoise");
  clock_ = node->get_clock();

  Params p;
  p.enable = node->get_parameter("denoise.enable").as_bool();
  p.num_threads = static_cast<int>(node->get_parameter("denoise.num_threads").as_int());

  p.clip_enable = node->get_parameter("denoise.clip.enable").as_bool();
  p.min_depth = static_cast<float>(node->get_parameter("denoise.clip.min_depth").as_double());
  p.max_depth = static_cast<float>(node->get_parameter("denoise.clip.max_depth").as_double());

  p.speckle_enable = node->get_parameter("denoise.speckle.enable").as_bool();
  p.speckle_max_size =
    static_cast<int>(node->get_parameter("denoise.speckle.max_size").as_int());
  p.speckle_diff_base =
    static_cast<float>(node->get_parameter("denoise.speckle.diff_base").as_double());
  p.speckle_diff_quad =
    static_cast<float>(node->get_parameter("denoise.speckle.diff_quad").as_double());

  p.bilateral_enable = node->get_parameter("denoise.bilateral.enable").as_bool();
  p.bilateral_radius =
    static_cast<int>(node->get_parameter("denoise.bilateral.radius").as_int());
  p.bilateral_sigma_space =
    static_cast<float>(node->get_parameter("denoise.bilateral.sigma_space").as_double());
  p.bilateral_sigma_base =
    static_cast<float>(node->get_parameter("denoise.bilateral.sigma_base").as_double());
  p.bilateral_sigma_quad =
    static_cast<float>(node->get_parameter("denoise.bilateral.sigma_quad").as_double());
  p.bilateral_min_valid =
    static_cast<int>(node->get_parameter("denoise.bilateral.min_valid").as_int());
  p.bilateral_invalidate_sparse =
    node->get_parameter("denoise.bilateral.invalidate_sparse").as_bool();

  p.temporal_enable = node->get_parameter("denoise.temporal.enable").as_bool();
  p.temporal_alpha =
    static_cast<float>(node->get_parameter("denoise.temporal.alpha").as_double());
  p.temporal_diff_base =
    static_cast<float>(node->get_parameter("denoise.temporal.diff_base").as_double());
  p.temporal_diff_quad =
    static_cast<float>(node->get_parameter("denoise.temporal.diff_quad").as_double());
  p.temporal_max_gap_sec = node->get_parameter("denoise.temporal.max_gap_sec").as_double();

  if (p.min_depth >= p.max_depth) {
    RCLCPP_WARN(
      logger_, "min_depth (%.3f) >= max_depth (%.3f); disabling the range clip.",
      static_cast<double>(p.min_depth), static_cast<double>(p.max_depth));
    p.clip_enable = false;
  }
  p.bilateral_radius = std::clamp(p.bilateral_radius, 0, 8);
  p.temporal_alpha = std::clamp(p.temporal_alpha, 0.0f, 1.0f);
  p.speckle_max_size = std::max(p.speckle_max_size, 0);

  configure(p);

  if (params_.num_threads > 0) {
    // Process-global, and this node normally runs inside a shared component
    // container, so only touch it when explicitly asked to.
    cv::setNumThreads(params_.num_threads);
    RCLCPP_WARN(
      logger_,
      "Set the OpenCV thread count to %d; this affects every component in this process.",
      params_.num_threads);
  }
}

void DepthDenoiser::configure(const Params & params)
{
  params_ = params;
  params_.bilateral_radius = std::clamp(params_.bilateral_radius, 0, 8);
  params_.temporal_alpha = std::clamp(params_.temporal_alpha, 0.0f, 1.0f);
  params_.speckle_max_size = std::max(params_.speckle_max_size, 0);
  buildTables();
  reset();
  configured_ = true;
}

void DepthDenoiser::buildTables()
{
  const int r = params_.bilateral_radius;
  const int side = 2 * r + 1;
  const float sigma_s = std::max(params_.bilateral_sigma_space, 1e-3f);
  const float inv2sig2 = 1.0f / (2.0f * sigma_s * sigma_s);

  spatial_w_.assign(static_cast<size_t>(side) * static_cast<size_t>(side), 0.0f);
  for (int dy = -r; dy <= r; ++dy) {
    for (int dx = -r; dx <= r; ++dx) {
      const float d2 = static_cast<float>(dx * dx + dy * dy);
      spatial_w_[static_cast<size_t>((dy + r) * side + (dx + r))] = std::exp(-d2 * inv2sig2);
    }
  }

  range_lut_.resize(kRangeLutSize);
  for (int i = 0; i < kRangeLutSize; ++i) {
    const float t = (static_cast<float>(i) + 0.5f) * kRangeCutoff /
      static_cast<float>(kRangeLutSize);
    range_lut_[static_cast<size_t>(i)] = std::exp(-t);
  }
  lut_scale_ = static_cast<float>(kRangeLutSize) / kRangeCutoff;
}

std::string DepthDenoiser::describe() const
{
  std::ostringstream os;
  os << "enable=" << (params_.enable ? "true" : "false");
  if (!params_.enable) {
    return os.str();
  }
  os << ", clip=";
  if (params_.clip_enable) {
    os << "[" << params_.min_depth << ", " << params_.max_depth << "]m";
  } else {
    os << "off";
  }
  os << ", speckle=";
  if (params_.speckle_enable && params_.speckle_max_size > 0) {
    os << "<" << params_.speckle_max_size << "px tol(z)=" << params_.speckle_diff_base
       << "+" << params_.speckle_diff_quad << "*z^2";
  } else {
    os << "off";
  }
  os << ", bilateral=";
  if (params_.bilateral_enable && params_.bilateral_radius > 0) {
    const int side = 2 * params_.bilateral_radius + 1;
    os << side << "x" << side << " sigma_r(z)=" << params_.bilateral_sigma_base
       << "+" << params_.bilateral_sigma_quad << "*z^2";
  } else {
    os << "off";
  }
  os << ", temporal=";
  if (params_.temporal_enable && params_.temporal_alpha > 0.0f) {
    os << "alpha=" << params_.temporal_alpha << " gate(z)=" << params_.temporal_diff_base
       << "+" << params_.temporal_diff_quad << "*z^2";
  } else {
    os << "off";
  }
  return os.str();
}

void DepthDenoiser::reset()
{
  prev_.release();
  prev_frame_id_.clear();
  prev_stamp_ns_ = 0;
  has_prev_ = false;
}

bool DepthDenoiser::apply(
  const sensor_msgs::msg::Image::ConstSharedPtr & in,
  sensor_msgs::msg::Image::SharedPtr & out)
{
  if (!configured_ || !params_.enable || !in) {
    return false;
  }

  bool is_16u = false;
  size_t pixel_bytes = 0;
  if (in->encoding == enc::TYPE_16UC1) {
    is_16u = true;
    pixel_bytes = sizeof(uint16_t);
  } else if (in->encoding == enc::TYPE_32FC1) {
    is_16u = false;
    pixel_bytes = sizeof(float);
  } else {
    // imageCb() reports the unsupported encoding itself; just step aside.
    return false;
  }

  if (in->width == 0 || in->height == 0) {
    return false;
  }
  if (in->is_bigendian != 0) {
    if (clock_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000, "Big-endian depth images are not supported; skipping denoise.");
    }
    return false;
  }
  const size_t min_step = static_cast<size_t>(in->width) * pixel_bytes;
  if (in->step < min_step ||
    in->data.size() < static_cast<size_t>(in->step) * static_cast<size_t>(in->height))
  {
    if (clock_) {
      RCLCPP_WARN_THROTTLE(
        logger_, *clock_, 5000,
        "Malformed depth image (step=%u, data=%zu, %ux%u); skipping denoise.",
        in->step, in->data.size(), in->width, in->height);
    }
    return false;
  }

  decode(*in, is_16u);

  if (params_.clip_enable) {
    applyClip();
  }
  if (params_.speckle_enable && params_.speckle_max_size > 0) {
    applySpeckle();
  }
  if (params_.bilateral_enable && params_.bilateral_radius > 0) {
    applyBilateral();
  }
  if (params_.temporal_enable && params_.temporal_alpha > 0.0f) {
    applyTemporal(stampToNanoseconds(in->header.stamp), in->header.frame_id);
  }

  out = std::make_shared<sensor_msgs::msg::Image>();
  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->encoding = in->encoding;
  out->is_bigendian = 0;
  out->step = static_cast<uint32_t>(min_step);
  out->data.resize(static_cast<size_t>(out->step) * static_cast<size_t>(out->height));
  encode(*out, is_16u);

  return true;
}

void DepthDenoiser::decode(const sensor_msgs::msg::Image & in, bool is_16u)
{
  const int rows = static_cast<int>(in.height);
  const int cols = static_cast<int>(in.width);
  work_.create(rows, cols, CV_32FC1);

  for (int y = 0; y < rows; ++y) {
    float * dst = work_.ptr<float>(y);
    const uint8_t * src_row = in.data.data() + static_cast<size_t>(y) * in.step;
    if (is_16u) {
      const uint16_t * src = reinterpret_cast<const uint16_t *>(src_row);
      for (int x = 0; x < cols; ++x) {
        // Matches DepthTraits<uint16_t>: 0 is the invalid marker, 1 count = 1 mm.
        dst[x] = (src[x] != 0) ? (static_cast<float>(src[x]) * 0.001f) : kNaN;
      }
    } else {
      const float * src = reinterpret_cast<const float *>(src_row);
      for (int x = 0; x < cols; ++x) {
        dst[x] = std::isfinite(src[x]) ? src[x] : kNaN;
      }
    }
  }
}

void DepthDenoiser::encode(sensor_msgs::msg::Image & out, bool is_16u) const
{
  const int rows = work_.rows;
  const int cols = work_.cols;

  for (int y = 0; y < rows; ++y) {
    const float * src = work_.ptr<float>(y);
    uint8_t * dst_row = out.data.data() + static_cast<size_t>(y) * out.step;
    if (is_16u) {
      uint16_t * dst = reinterpret_cast<uint16_t *>(dst_row);
      for (int x = 0; x < cols; ++x) {
        dst[x] = metersToMillimeters(src[x]);
      }
    } else {
      float * dst = reinterpret_cast<float *>(dst_row);
      std::memcpy(dst, src, static_cast<size_t>(cols) * sizeof(float));
    }
  }
}

void DepthDenoiser::applyClip()
{
  const int rows = work_.rows;
  const int cols = work_.cols;
  const float lo = params_.min_depth;
  const float hi = params_.max_depth;

  for (int y = 0; y < rows; ++y) {
    float * row = work_.ptr<float>(y);
    for (int x = 0; x < cols; ++x) {
      const float z = row[x];
      if (!std::isfinite(z) || z < lo || z > hi) {
        row[x] = kNaN;
      }
    }
  }
}

void DepthDenoiser::applySpeckle()
{
  const int rows = work_.rows;
  const int cols = work_.cols;

  // work_ comes from decode(), so it is continuous and can be walked flat.
  float * data = work_.ptr<float>(0);

  const int max_size = params_.speckle_max_size;
  const float base = params_.speckle_diff_base;
  const float quad = params_.speckle_diff_quad;

  run_row_.clear();
  run_x0_.clear();
  run_x1_.clear();
  uf_parent_.clear();
  uf_size_.clear();
  row_run_begin_.assign(static_cast<size_t>(rows) + 1, 0);

  // Pass 1: split every row into maximal runs of depth-continuous valid pixels.
  for (int y = 0; y < rows; ++y) {
    row_run_begin_[static_cast<size_t>(y)] = static_cast<int32_t>(run_row_.size());
    const float * row = data + static_cast<size_t>(y) * cols;
    int x = 0;
    while (x < cols) {
      if (!std::isfinite(row[x])) {
        ++x;
        continue;
      }
      const int x0 = x;
      while (x + 1 < cols && std::isfinite(row[x + 1]) &&
        std::fabs(row[x + 1] - row[x]) <= toleranceAt(row[x], base, quad))
      {
        ++x;
      }
      const int32_t id = static_cast<int32_t>(run_row_.size());
      run_row_.push_back(y);
      run_x0_.push_back(x0);
      run_x1_.push_back(x);
      uf_parent_.push_back(id);
      uf_size_.push_back(static_cast<int32_t>(x - x0 + 1));
      ++x;
    }
  }
  row_run_begin_[static_cast<size_t>(rows)] = static_cast<int32_t>(run_row_.size());

  // Pass 2: join runs in adjacent rows, 8-connected. Both run lists are sorted by
  // x, so a two-pointer sweep visits each pair of overlapping runs once.
  for (int y = 1; y < rows; ++y) {
    const float * above = data + static_cast<size_t>(y - 1) * cols;
    const float * below = data + static_cast<size_t>(y) * cols;
    int32_t ia = row_run_begin_[static_cast<size_t>(y - 1)];
    int32_t ib = row_run_begin_[static_cast<size_t>(y)];
    const int32_t ia_end = row_run_begin_[static_cast<size_t>(y)];
    const int32_t ib_end = row_run_begin_[static_cast<size_t>(y) + 1];

    while (ia < ia_end && ib < ib_end) {
      const int ax0 = run_x0_[static_cast<size_t>(ia)];
      const int ax1 = run_x1_[static_cast<size_t>(ia)];
      const int bx0 = run_x0_[static_cast<size_t>(ib)];
      const int bx1 = run_x1_[static_cast<size_t>(ib)];

      // Diagonal neighbours count, so the spans overlap when widened by one.
      const int lo = std::max(ax0 - 1, bx0);
      const int hi = std::min(ax1 + 1, bx1);
      if (lo <= hi) {
        bool joined = false;
        for (int xb = lo; xb <= hi && !joined; ++xb) {
          const float zb = below[xb];
          const int xa_lo = std::max(xb - 1, ax0);
          const int xa_hi = std::min(xb + 1, ax1);
          for (int xa = xa_lo; xa <= xa_hi; ++xa) {
            const float za = above[xa];
            if (std::fabs(zb - za) <= toleranceAt(za, base, quad)) {
              ufUnion(uf_parent_, uf_size_, ia, ib);
              joined = true;
              break;
            }
          }
        }
      }

      if (ax1 < bx1) {
        ++ia;
      } else {
        ++ib;
      }
    }
  }

  // Pass 3: blank out every run whose component is too small to be real geometry.
  const size_t run_count = run_row_.size();
  for (size_t i = 0; i < run_count; ++i) {
    const int32_t root = ufFind(uf_parent_, static_cast<int32_t>(i));
    if (uf_size_[static_cast<size_t>(root)] >= max_size) {
      continue;
    }
    float * row = data + static_cast<size_t>(run_row_[i]) * cols;
    for (int x = run_x0_[i]; x <= run_x1_[i]; ++x) {
      row[x] = kNaN;
    }
  }
}

void DepthDenoiser::applyBilateral()
{
  const int rows = work_.rows;
  const int cols = work_.cols;
  const int r = params_.bilateral_radius;
  const int side = 2 * r + 1;
  const int min_valid = params_.bilateral_min_valid;
  const bool invalidate_sparse = params_.bilateral_invalidate_sparse;
  const float sigma_base = params_.bilateral_sigma_base;
  const float sigma_quad = params_.bilateral_sigma_quad;
  const float * spatial = spatial_w_.data();
  const float * lut = range_lut_.data();
  const float lut_scale = lut_scale_;

  tmp_.create(rows, cols, CV_32FC1);

  cv::parallel_for_(
    cv::Range(0, rows), [&](const cv::Range & range) {
      for (int y = range.start; y < range.end; ++y) {
        const float * src_row = work_.ptr<float>(y);
        float * dst_row = tmp_.ptr<float>(y);

        const int y0 = std::max(y - r, 0);
        const int y1 = std::min(y + r, rows - 1);

        for (int x = 0; x < cols; ++x) {
          const float z0 = src_row[x];
          if (!std::isfinite(z0)) {
            // Never invent depth inside a hole.
            dst_row[x] = kNaN;
            continue;
          }

          // Stereo error grows as z^2, so the range sigma has to follow it.
          const float sigma = toleranceAt(z0, sigma_base, sigma_quad);
          const float inv2sig2 = 1.0f / (2.0f * sigma * sigma);

          const int x0 = std::max(x - r, 0);
          const int x1 = std::min(x + r, cols - 1);

          float wsum = 0.0f;
          float zsum = 0.0f;
          int nvalid = 0;

          for (int ny = y0; ny <= y1; ++ny) {
            const float * nrow = work_.ptr<float>(ny);
            const int krow = (ny - y + r) * side;
            for (int nx = x0; nx <= x1; ++nx) {
              const float zn = nrow[nx];
              if (!std::isfinite(zn)) {
                continue;
              }
              const float d = zn - z0;
              const float t = d * d * inv2sig2;
              if (t >= kRangeCutoff) {
                continue;
              }
              const int li = static_cast<int>(t * lut_scale);
              const float w = spatial[krow + (nx - x + r)] * lut[li];
              wsum += w;
              zsum += w * zn;
              ++nvalid;
            }
          }

          if (nvalid >= min_valid && wsum > 0.0f) {
            dst_row[x] = zsum / wsum;
          } else {
            // Too few neighbours to average: keep the pixel rather than eroding
            // thin structures a little further every frame.
            dst_row[x] = invalidate_sparse ? kNaN : z0;
          }
        }
      }
    });

  cv::swap(work_, tmp_);
}

void DepthDenoiser::applyTemporal(int64_t stamp_ns, const std::string & frame_id)
{
  const int rows = work_.rows;
  const int cols = work_.cols;

  if (has_prev_) {
    const int64_t dt = stamp_ns - prev_stamp_ns_;
    const int64_t max_dt =
      static_cast<int64_t>(params_.temporal_max_gap_sec * 1e9);
    if (prev_.rows != rows || prev_.cols != cols || frame_id != prev_frame_id_ ||
      dt < 0 || dt > max_dt)
    {
      // Resolution or camera change, a bag loop, or a stale stream.
      reset();
    }
  }

  if (!has_prev_) {
    work_.copyTo(prev_);
    prev_stamp_ns_ = stamp_ns;
    prev_frame_id_ = frame_id;
    has_prev_ = true;
    return;
  }

  const float alpha = params_.temporal_alpha;
  const float one_minus = 1.0f - alpha;
  const float base = params_.temporal_diff_base;
  const float quad = params_.temporal_diff_quad;

  for (int y = 0; y < rows; ++y) {
    float * cur = work_.ptr<float>(y);
    float * prev = prev_.ptr<float>(y);
    for (int x = 0; x < cols; ++x) {
      const float c = cur[x];
      const float p = prev[x];
      float value;
      if (!std::isfinite(c)) {
        // Never hole-fill from the previous frame: that is how ghosts appear.
        value = kNaN;
      } else if (!std::isfinite(p)) {
        value = c;
      } else if (std::fabs(c - p) > toleranceAt(c, base, quad)) {
        // Real motion, not noise. Restart the average from the current sample.
        value = c;
      } else {
        value = alpha * c + one_minus * p;
      }
      cur[x] = value;
      prev[x] = value;
    }
  }

  prev_stamp_ns_ = stamp_ns;
  prev_frame_id_ = frame_id;
}

}  // namespace hma_pcl_reconst2
