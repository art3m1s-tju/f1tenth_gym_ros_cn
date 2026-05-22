#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <limits>
#include <memory>
#include <filesystem>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "nav_msgs/msg/path.hpp"
#include "pnc_rc/lqr_core.h"
#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2/time.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace {

struct Point2 {
  double x = 0.0;
  double y = 0.0;
};

struct ReferenceSample {
  Point2 point;
  double heading = 0.0;
  double curvature = 0.0;
  int segment_idx = 0;
  double segment_t = 0.0;
  int closest_idx = 0;
};

struct GainEntry {
  double speed = 0.0;
  double q_lateral = 3.0;
  double q_heading = 1.2;
  double r_steering = 8.0;
  double feedforward_gain = 1.0;
};

double norm(Point2 point) {
  return std::hypot(point.x, point.y);
}

Point2 subtract(Point2 lhs, Point2 rhs) {
  return {lhs.x - rhs.x, lhs.y - rhs.y};
}

double dot(Point2 lhs, Point2 rhs) {
  return lhs.x * rhs.x + lhs.y * rhs.y;
}

double interpolate_angle(double start_angle, double end_angle, double t) {
  return pnc_wrap_angle(start_angle + t * pnc_wrap_angle(end_angle - start_angle));
}

double quaternion_to_yaw(double x, double y, double z, double w) {
  return std::atan2(
      2.0 * (w * z + x * y),
      1.0 - 2.0 * (y * y + z * z));
}

std::vector<double> compute_path_headings(
    const std::vector<Point2> &points,
    bool closed_loop) {
  std::vector<double> headings(points.size(), 0.0);
  for (size_t idx = 0; idx < points.size(); ++idx) {
    Point2 previous;
    Point2 next;
    if (closed_loop) {
      previous = points[(idx + points.size() - 1) % points.size()];
      next = points[(idx + 1) % points.size()];
    } else if (idx == 0) {
      previous = points[idx];
      next = points[idx + 1];
    } else if (idx + 1 == points.size()) {
      previous = points[idx - 1];
      next = points[idx];
    } else {
      previous = points[idx - 1];
      next = points[idx + 1];
    }

    Point2 tangent = subtract(next, previous);
    if (norm(tangent) <= 1e-9) {
      tangent = {1.0, 0.0};
    }
    headings[idx] = std::atan2(tangent.y, tangent.x);
  }
  return headings;
}

std::vector<double> compute_path_curvatures(
    const std::vector<Point2> &points,
    bool closed_loop) {
  std::vector<double> curvatures(points.size(), 0.0);
  if (points.size() < 3) {
    return curvatures;
  }

  for (size_t idx = 0; idx < points.size(); ++idx) {
    if (!closed_loop && (idx == 0 || idx + 1 == points.size())) {
      continue;
    }

    const Point2 previous = points[(idx + points.size() - 1) % points.size()];
    const Point2 current = points[idx];
    const Point2 next = points[(idx + 1) % points.size()];
    const Point2 first_chord = subtract(current, previous);
    const Point2 second_chord = subtract(next, current);
    const Point2 full_chord = subtract(next, previous);
    const double denominator =
        norm(first_chord) * norm(second_chord) * norm(full_chord);
    if (denominator <= 1e-9) {
      continue;
    }

    const double cross_z =
        first_chord.x * second_chord.y - first_chord.y * second_chord.x;
    curvatures[idx] = 2.0 * cross_z / denominator;
  }

  if (!closed_loop) {
    curvatures.front() = curvatures[1];
    curvatures.back() = curvatures[curvatures.size() - 2];
  }
  return curvatures;
}

std::vector<double> compute_segment_lengths(
    const std::vector<Point2> &points,
    bool closed_loop) {
  const size_t segment_count = closed_loop ? points.size() : points.size() - 1;
  std::vector<double> lengths(segment_count, 0.0);
  for (size_t idx = 0; idx < segment_count; ++idx) {
    lengths[idx] = norm(subtract(points[(idx + 1) % points.size()], points[idx]));
  }
  return lengths;
}

std::string build_path_signature(
    const std::vector<Point2> &points,
    const std::string &frame_id,
    bool closed_loop) {
  std::string signature = frame_id + "|" + (closed_loop ? "1" : "0") + "|";
  signature += std::to_string(points.size()) + "|";
  for (const auto &point : points) {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "%.6f,%.6f;", point.x, point.y);
    signature += buffer;
  }
  return signature;
}

double yaml_value(const std::string &line) {
  const size_t colon = line.find(':');
  if (colon == std::string::npos) {
    return 0.0;
  }
  return std::stod(line.substr(colon + 1));
}

}  // namespace

class LqrControllerNode : public rclcpp::Node {
 public:
  LqrControllerNode()
      : Node("lqr_controller"),
        tf_buffer_(get_clock()),
        tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_)) {
    declare_parameters();
    read_parameters();
    open_tracking_log();

    rclcpp::QoS path_qos(1);
    path_qos.transient_local();
    path_sub_ = create_subscription<nav_msgs::msg::Path>(
        path_topic_, path_qos,
        std::bind(&LqrControllerNode::path_callback, this, std::placeholders::_1));
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_, 10,
        std::bind(&LqrControllerNode::odom_callback, this, std::placeholders::_1));
    drive_pub_ = create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic_, 10);
    tracked_path_pub_ = create_publisher<nav_msgs::msg::Path>(
        tracked_path_topic_, 10);

    RCLCPP_INFO(
        get_logger(),
        "C LQR controller started (path_topic=%s, odom_topic=%s, drive_topic=%s, "
        "use_tf_pose=%s, target_speed=%.2f, Q=(%.2f, %.2f), R=%.2f).",
        path_topic_.c_str(),
        odom_topic_.c_str(),
        drive_topic_.c_str(),
        use_tf_pose_ ? "true" : "false",
        target_speed_,
        lqr_q_lateral_,
        lqr_q_heading_,
        lqr_r_steering_);
  }

 private:
  void declare_parameters() {
    declare_parameter("path_topic", "/global_trajectory");
    declare_parameter("odom_topic", "/odom");
    declare_parameter("drive_topic", "/drive");
    declare_parameter("tracked_frame", "");
    declare_parameter("use_tf_pose", true);
    declare_parameter("vehicle_frame", "base_link");
    declare_parameter("tf_lookup_timeout_sec", 0.02);
    declare_parameter("path_closed_loop", true);
    declare_parameter("open_loop_finish_distance", 0.25);
    declare_parameter("wheelbase", 0.3302);
    declare_parameter("target_speed", 0.60);
    declare_parameter("min_speed", 0.25);
    declare_parameter("max_steering_angle", 0.61);
    declare_parameter("max_lateral_accel", 3.0);
    declare_parameter("enable_curvature_speed_limit", true);
    declare_parameter("curvature_speed_lookahead_m", 1.0);
    declare_parameter("enable_speed_ramp", true);
    declare_parameter("max_accel", 0.8);
    declare_parameter("max_decel", 1.5);
    declare_parameter("enable_steering_rate_limit", true);
    declare_parameter("max_steering_rate", 2.0);
    declare_parameter("validation_position_noise_std", 0.0);
    declare_parameter("validation_heading_noise_std_deg", 0.0);
    declare_parameter("validation_pose_delay_ms", 0.0);
    declare_parameter("validation_noise_seed", 42);
    declare_parameter("enable_error_filter", false);
    declare_parameter("error_filter_alpha_y", 0.30);
    declare_parameter("error_filter_alpha_psi", 0.25);
    declare_parameter("control_dt", 0.05);
    declare_parameter("lqr_min_model_speed", 0.25);
    declare_parameter("lqr_lookahead_distance_m", 1.5);
    declare_parameter("lqr_q_lateral", 3.0);
    declare_parameter("lqr_q_heading", 1.2);
    declare_parameter("lqr_r_steering", 8.0);
    declare_parameter("lqr_feedforward_gain", 1.0);
    declare_parameter("lqr_gain_table_path", "");
    declare_parameter("enable_tracking_csv_log", true);
    declare_parameter("tracking_log_path", "outputs/logs/lqr_tracking_log.csv");
    declare_parameter("tracked_path_topic", "/tracked_path_lqr");
    declare_parameter("max_tracked_path_points", 2000);
  }

  void read_parameters() {
    path_topic_ = get_parameter("path_topic").as_string();
    odom_topic_ = get_parameter("odom_topic").as_string();
    drive_topic_ = get_parameter("drive_topic").as_string();
    tracked_frame_ = get_parameter("tracked_frame").as_string();
    use_tf_pose_ = get_parameter("use_tf_pose").as_bool();
    vehicle_frame_ = get_parameter("vehicle_frame").as_string();
    tf_lookup_timeout_sec_ =
        std::max(0.0, get_parameter("tf_lookup_timeout_sec").as_double());
    path_closed_loop_ = get_parameter("path_closed_loop").as_bool();
    open_loop_finish_distance_ =
        std::max(1e-3, get_parameter("open_loop_finish_distance").as_double());
    wheelbase_ = std::max(1e-6, get_parameter("wheelbase").as_double());
    target_speed_ = std::max(0.0, get_parameter("target_speed").as_double());
    min_speed_ = std::max(0.0, get_parameter("min_speed").as_double());
    max_steering_angle_ =
        std::max(1e-6, get_parameter("max_steering_angle").as_double());
    max_lateral_accel_ =
        std::max(1e-6, get_parameter("max_lateral_accel").as_double());
    enable_curvature_speed_limit_ =
        get_parameter("enable_curvature_speed_limit").as_bool();
    curvature_speed_lookahead_m_ =
        std::max(0.0, get_parameter("curvature_speed_lookahead_m").as_double());
    enable_speed_ramp_ = get_parameter("enable_speed_ramp").as_bool();
    max_accel_ = std::max(0.0, get_parameter("max_accel").as_double());
    max_decel_ = std::max(0.0, get_parameter("max_decel").as_double());
    enable_steering_rate_limit_ =
        get_parameter("enable_steering_rate_limit").as_bool();
    max_steering_rate_ =
        std::max(0.0, get_parameter("max_steering_rate").as_double());
    enable_error_filter_ = get_parameter("enable_error_filter").as_bool();
    error_filter_alpha_y_ =
        std::clamp(get_parameter("error_filter_alpha_y").as_double(), 0.0, 1.0);
    error_filter_alpha_psi_ =
        std::clamp(get_parameter("error_filter_alpha_psi").as_double(), 0.0, 1.0);
    control_dt_ = std::max(1e-4, get_parameter("control_dt").as_double());
    lqr_min_model_speed_ =
        std::max(1e-4, get_parameter("lqr_min_model_speed").as_double());
    lqr_lookahead_distance_m_ =
        std::max(0.0, get_parameter("lqr_lookahead_distance_m").as_double());
    lqr_q_lateral_ = std::max(1e-9, get_parameter("lqr_q_lateral").as_double());
    lqr_q_heading_ = std::max(1e-9, get_parameter("lqr_q_heading").as_double());
    lqr_r_steering_ = std::max(1e-9, get_parameter("lqr_r_steering").as_double());
    lqr_feedforward_gain_ = get_parameter("lqr_feedforward_gain").as_double();
    enable_tracking_csv_log_ = get_parameter("enable_tracking_csv_log").as_bool();
    tracking_log_path_ = get_parameter("tracking_log_path").as_string();
    tracked_path_topic_ = get_parameter("tracked_path_topic").as_string();
    max_tracked_path_points_ =
        std::max<int64_t>(1, get_parameter("max_tracked_path_points").as_int());

    const auto gain_table_path = get_parameter("lqr_gain_table_path").as_string();
    if (!gain_table_path.empty()) {
      load_gain_table(gain_table_path);
    }
  }

  void load_gain_table(const std::string &path) {
    std::ifstream file(path);
    if (!file.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open LQR gain table: %s", path.c_str());
      return;
    }

    std::vector<GainEntry> entries;
    GainEntry current;
    bool in_entry = false;
    std::string line;
    while (std::getline(file, line)) {
      if (line.find("- speed:") != std::string::npos) {
        if (in_entry) {
          entries.push_back(current);
        }
        current = GainEntry();
        current.speed = yaml_value(line);
        in_entry = true;
      } else if (in_entry && line.find("q_lateral:") != std::string::npos) {
        current.q_lateral = yaml_value(line);
      } else if (in_entry && line.find("q_heading:") != std::string::npos) {
        current.q_heading = yaml_value(line);
      } else if (in_entry && line.find("r_steering:") != std::string::npos) {
        current.r_steering = yaml_value(line);
      } else if (in_entry && line.find("feedforward_gain:") != std::string::npos) {
        current.feedforward_gain = yaml_value(line);
      }
    }
    if (in_entry) {
      entries.push_back(current);
    }

    if (entries.empty()) {
      RCLCPP_WARN(get_logger(), "LQR gain table has no entries: %s", path.c_str());
      return;
    }

    std::sort(
        entries.begin(),
        entries.end(),
        [](const GainEntry &lhs, const GainEntry &rhs) {
          return lhs.speed < rhs.speed;
        });
    gain_table_ = std::move(entries);
    RCLCPP_INFO(
        get_logger(),
        "Loaded C LQR gain table with %zu speed points.",
        gain_table_.size());
  }

  void open_tracking_log() {
    if (!enable_tracking_csv_log_) {
      return;
    }
    const auto log_path = std::filesystem::path(tracking_log_path_);
    if (!log_path.parent_path().empty()) {
      std::filesystem::create_directories(log_path.parent_path());
    }
    csv_file_.open(tracking_log_path_, std::ios::out | std::ios::trunc);
    if (!csv_file_.is_open()) {
      RCLCPP_WARN(get_logger(), "Failed to open tracking log: %s", tracking_log_path_.c_str());
      return;
    }
    csv_file_
        << "time,x,y,yaw,v_actual,v_cmd,e_y,e_psi,filtered_e_y,filtered_e_psi,"
        << "curvature_ref,curvature_preview,delta_raw,delta_rate_limited,"
        << "delta_feedback,delta_feedforward,delta_cmd,lqr_k_y,lqr_k_psi,"
        << "closest_idx,segment_idx,segment_t,compute_time_ms\n";
  }

  void path_callback(const nav_msgs::msg::Path::SharedPtr msg) {
    std::vector<Point2> points;
    points.reserve(msg->poses.size());
    for (const auto &pose : msg->poses) {
      points.push_back({pose.pose.position.x, pose.pose.position.y});
    }
    if (points.size() < 2) {
      RCLCPP_WARN(get_logger(), "Ignoring path with fewer than 2 poses.");
      return;
    }

    const std::string frame_id =
        msg->header.frame_id.empty()
            ? (tracked_frame_.empty() ? "map" : tracked_frame_)
            : msg->header.frame_id;
    const std::string signature = build_path_signature(points, frame_id, path_closed_loop_);
    if (signature == path_signature_) {
      return;
    }

    points_ = std::move(points);
    headings_ = compute_path_headings(points_, path_closed_loop_);
    curvatures_ = compute_path_curvatures(points_, path_closed_loop_);
    segment_lengths_ = compute_segment_lengths(points_, path_closed_loop_);
    path_frame_id_ = frame_id;
    path_signature_ = signature;
    open_loop_finished_ = false;
    previous_delta_cmd_ = 0.0;
    filtered_lateral_error_.reset();
    filtered_heading_error_.reset();
    tracked_path_msg_ = nav_msgs::msg::Path();
    tracked_path_msg_.header.frame_id = path_frame_id_;

    RCLCPP_INFO(
        get_logger(),
        "Loaded C LQR reference path with %zu points (frame=%s, closed_loop=%s).",
        points_.size(),
        path_frame_id_.c_str(),
        path_closed_loop_ ? "true" : "false");
  }

  std::optional<std::pair<Point2, double>> resolve_pose(
      const nav_msgs::msg::Odometry::SharedPtr msg) {
    if (use_tf_pose_) {
      try {
        const auto transform = tf_buffer_.lookupTransform(
            path_frame_id_,
            vehicle_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(tf_lookup_timeout_sec_));
        const auto &translation = transform.transform.translation;
        const auto &rotation = transform.transform.rotation;
        return std::make_pair(
            Point2{translation.x, translation.y},
            quaternion_to_yaw(rotation.x, rotation.y, rotation.z, rotation.w));
      } catch (const tf2::TransformException &exc) {
        RCLCPP_WARN_THROTTLE(
            get_logger(),
            *get_clock(),
            1000,
            "Skipping LQR update because TF pose is unavailable: %s",
            exc.what());
        return std::nullopt;
      }
    }

    const auto &pose = msg->pose.pose;
    const std::string expected_frame = tracked_frame_.empty() ? path_frame_id_ : tracked_frame_;
    if (!msg->header.frame_id.empty() && !expected_frame.empty() &&
        msg->header.frame_id != expected_frame) {
      RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Using odometry pose even though odom frame '%s' differs from path frame '%s'.",
          msg->header.frame_id.c_str(),
          expected_frame.c_str());
    }
    return std::make_pair(
        Point2{pose.position.x, pose.position.y},
        quaternion_to_yaw(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w));
  }

  ReferenceSample project_to_path(Point2 position) const {
    const int point_count = static_cast<int>(points_.size());
    const int segment_count = path_closed_loop_ ? point_count : point_count - 1;
    double best_distance = std::numeric_limits<double>::infinity();
    ReferenceSample best;

    for (int segment_idx = 0; segment_idx < segment_count; ++segment_idx) {
      const int next_idx = (segment_idx + 1) % point_count;
      const Point2 start = points_[segment_idx];
      const Point2 end = points_[next_idx];
      const Point2 segment = subtract(end, start);
      const double segment_length_sq = dot(segment, segment);
      if (segment_length_sq <= 1e-12) {
        continue;
      }

      const double t = std::clamp(
          dot(subtract(position, start), segment) / segment_length_sq,
          0.0,
          1.0);
      const Point2 projected = {
          start.x + t * segment.x,
          start.y + t * segment.y};
      const double distance = norm(subtract(position, projected));
      if (distance >= best_distance) {
        continue;
      }

      best_distance = distance;
      best.point = projected;
      best.segment_idx = segment_idx;
      best.segment_t = t;
      best.closest_idx = t < 0.5 ? segment_idx : next_idx;
      best.heading = interpolate_angle(headings_[segment_idx], headings_[next_idx], t);
      best.curvature = (1.0 - t) * curvatures_[segment_idx] + t * curvatures_[next_idx];
    }
    return advance_projection(position, best);
  }

  ReferenceSample advance_projection(Point2 position, ReferenceSample sample) const {
    if (lqr_lookahead_distance_m_ <= 1e-9 || points_.size() < 2) {
      return sample;
    }

    const int point_count = static_cast<int>(points_.size());
    const int segment_count = path_closed_loop_ ? point_count : point_count - 1;
    int segment_idx = std::clamp(sample.segment_idx, 0, segment_count - 1);
    double segment_t = std::clamp(sample.segment_t, 0.0, 1.0);
    double distance_left = lqr_lookahead_distance_m_;

    while (distance_left > 1e-9) {
      const double segment_length = segment_lengths_[segment_idx];
      if (segment_length <= 1e-9) {
        const int next_segment = next_segment_index(segment_idx, segment_count);
        if (next_segment == segment_idx) {
          break;
        }
        segment_idx = next_segment;
        segment_t = 0.0;
        continue;
      }

      const double remaining_segment = (1.0 - segment_t) * segment_length;
      if (remaining_segment <= 1e-9) {
        const int next_segment = next_segment_index(segment_idx, segment_count);
        if (next_segment == segment_idx) {
          segment_t = 1.0;
          break;
        }
        segment_idx = next_segment;
        segment_t = 0.0;
        continue;
      }
      if (distance_left <= remaining_segment) {
        segment_t = std::min(1.0, segment_t + distance_left / segment_length);
        break;
      }
      distance_left -= remaining_segment;
      const int next_segment = next_segment_index(segment_idx, segment_count);
      if (next_segment == segment_idx) {
        segment_t = 1.0;
        break;
      }
      segment_idx = next_segment;
      segment_t = 0.0;
    }

    int next_idx = (segment_idx + 1) % point_count;
    if (!path_closed_loop_) {
      next_idx = std::min(segment_idx + 1, point_count - 1);
    }

    const Point2 start = points_[segment_idx];
    const Point2 end = points_[next_idx];
    sample.point = {
        start.x + segment_t * (end.x - start.x),
        start.y + segment_t * (end.y - start.y)};
    sample.heading = interpolate_angle(headings_[segment_idx], headings_[next_idx], segment_t);
    sample.curvature =
        (1.0 - segment_t) * curvatures_[segment_idx] + segment_t * curvatures_[next_idx];
    sample.segment_idx = segment_idx;
    sample.segment_t = segment_t;
    sample.closest_idx = segment_t < 0.5 ? segment_idx : next_idx;
    (void)position;
    return sample;
  }

  int next_segment_index(int segment_idx, int segment_count) const {
    if (path_closed_loop_) {
      return (segment_idx + 1) % segment_count;
    }
    return std::min(segment_idx + 1, segment_count - 1);
  }

  std::pair<double, double> filter_control_errors(
      double lateral_error,
      double heading_error) {
    if (!enable_error_filter_) {
      filtered_lateral_error_.reset();
      filtered_heading_error_.reset();
      return {lateral_error, heading_error};
    }

    if (!filtered_lateral_error_ || !filtered_heading_error_) {
      filtered_lateral_error_ = lateral_error;
      filtered_heading_error_ = heading_error;
      return {lateral_error, heading_error};
    }

    filtered_lateral_error_ =
        error_filter_alpha_y_ * lateral_error +
        (1.0 - error_filter_alpha_y_) * *filtered_lateral_error_;
    const double heading_delta = pnc_wrap_angle(heading_error - *filtered_heading_error_);
    filtered_heading_error_ = pnc_wrap_angle(
        *filtered_heading_error_ + error_filter_alpha_psi_ * heading_delta);
    return {*filtered_lateral_error_, *filtered_heading_error_};
  }

  double preview_curvature(ReferenceSample sample) const {
    if (curvature_speed_lookahead_m_ <= 0.0 || curvatures_.empty()) {
      return std::abs(sample.curvature);
    }

    double max_curvature = std::abs(sample.curvature);
    double distance_left = curvature_speed_lookahead_m_;
    int segment_idx = sample.segment_idx;
    double segment_t = std::clamp(sample.segment_t, 0.0, 1.0);

    while (distance_left > 0.0) {
      int next_idx = 0;
      double segment_length = 0.0;
      if (path_closed_loop_) {
        segment_length = segment_lengths_[segment_idx];
        next_idx = (segment_idx + 1) % static_cast<int>(curvatures_.size());
      } else if (segment_idx >= static_cast<int>(segment_lengths_.size())) {
        break;
      } else {
        segment_length = segment_lengths_[segment_idx];
        next_idx = std::min(segment_idx + 1, static_cast<int>(curvatures_.size()) - 1);
      }

      const double remaining_segment = std::max(0.0, (1.0 - segment_t) * segment_length);
      max_curvature = std::max(max_curvature, std::abs(curvatures_[next_idx]));
      distance_left -= remaining_segment;

      if (remaining_segment <= 1e-9 && segment_t <= 0.0) {
        break;
      }
      if (!path_closed_loop_ && next_idx >= static_cast<int>(curvatures_.size()) - 1) {
        break;
      }
      segment_idx = next_idx;
      segment_t = 0.0;
    }
    return max_curvature;
  }

  std::pair<double, bool> apply_steering_rate_limit(double delta_cmd, double dt) {
    if (!enable_steering_rate_limit_ || max_steering_rate_ <= 0.0 || dt <= 0.0) {
      previous_delta_cmd_ = delta_cmd;
      return {delta_cmd, false};
    }

    const double max_delta_step = max_steering_rate_ * dt;
    const double delta_step =
        std::clamp(delta_cmd - previous_delta_cmd_, -max_delta_step, max_delta_step);
    const double limited_delta = previous_delta_cmd_ + delta_step;
    previous_delta_cmd_ = limited_delta;
    return {limited_delta, std::abs(limited_delta - delta_cmd) > 1e-9};
  }

  GainEntry interpolate_gain(double speed) const {
    if (gain_table_.empty()) {
      return {
          speed,
          lqr_q_lateral_,
          lqr_q_heading_,
          lqr_r_steering_,
          lqr_feedforward_gain_};
    }
    if (speed <= gain_table_.front().speed) {
      return gain_table_.front();
    }
    if (speed >= gain_table_.back().speed) {
      return gain_table_.back();
    }

    for (size_t idx = 0; idx + 1 < gain_table_.size(); ++idx) {
      const GainEntry &lower = gain_table_[idx];
      const GainEntry &upper = gain_table_[idx + 1];
      if (speed < lower.speed || speed > upper.speed) {
        continue;
      }
      const double span = std::max(upper.speed - lower.speed, 1e-9);
      const double t = std::clamp((speed - lower.speed) / span, 0.0, 1.0);
      return {
          speed,
          (1.0 - t) * lower.q_lateral + t * upper.q_lateral,
          (1.0 - t) * lower.q_heading + t * upper.q_heading,
          (1.0 - t) * lower.r_steering + t * upper.r_steering,
          (1.0 - t) * lower.feedforward_gain + t * upper.feedforward_gain};
    }
    return gain_table_.back();
  }

  double apply_speed_ramp(double desired_speed, double stamp_sec) {
    if (!enable_speed_ramp_) {
      current_speed_cmd_ = desired_speed;
      return desired_speed;
    }
    if (!previous_stamp_sec_) {
      current_speed_cmd_ = std::min(current_speed_cmd_, desired_speed);
      return current_speed_cmd_;
    }

    const double dt = std::max(0.0, stamp_sec - *previous_stamp_sec_);
    if (desired_speed >= current_speed_cmd_) {
      current_speed_cmd_ = std::min(desired_speed, current_speed_cmd_ + max_accel_ * dt);
    } else {
      current_speed_cmd_ = std::max(desired_speed, current_speed_cmd_ - max_decel_ * dt);
    }
    return current_speed_cmd_;
  }

  bool open_loop_finished(Point2 position) const {
    if (path_closed_loop_ || points_.empty()) {
      return false;
    }
    return norm(subtract(position, points_.back())) <= open_loop_finish_distance_;
  }

  void publish_drive(
      const builtin_interfaces::msg::Time &stamp,
      double steering,
      double speed) {
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = stamp;
    drive_msg.drive.steering_angle = steering;
    drive_msg.drive.speed = speed;
    drive_pub_->publish(drive_msg);
  }

  void append_tracked_pose(
      Point2 position,
      double yaw,
      const builtin_interfaces::msg::Time &stamp) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = path_frame_id_;
    pose.pose.position.x = position.x;
    pose.pose.position.y = position.y;
    pose.pose.orientation.z = std::sin(yaw * 0.5);
    pose.pose.orientation.w = std::cos(yaw * 0.5);

    tracked_path_msg_.header.stamp = stamp;
    tracked_path_msg_.header.frame_id = path_frame_id_;
    tracked_path_msg_.poses.push_back(pose);
    if (static_cast<int64_t>(tracked_path_msg_.poses.size()) > max_tracked_path_points_) {
      tracked_path_msg_.poses.erase(
          tracked_path_msg_.poses.begin(),
          tracked_path_msg_.poses.end() - max_tracked_path_points_);
    }
    tracked_path_pub_->publish(tracked_path_msg_);
  }

  void write_tracking_log(
      double stamp_sec,
      Point2 position,
      double yaw,
      double v_actual,
      double v_cmd,
      double lateral_error,
      double heading_error,
      double filtered_lateral_error,
      double filtered_heading_error,
      ReferenceSample sample,
      double curvature_preview,
      double delta_raw,
      bool delta_rate_limited,
      double delta_feedback,
      double delta_feedforward,
      double delta_cmd,
      double gain_y,
      double gain_psi,
      double compute_time_ms) {
    if (!csv_file_.is_open()) {
      return;
    }
    csv_file_
        << stamp_sec << ','
        << position.x << ','
        << position.y << ','
        << yaw << ','
        << v_actual << ','
        << v_cmd << ','
        << lateral_error << ','
        << heading_error << ','
        << filtered_lateral_error << ','
        << filtered_heading_error << ','
        << sample.curvature << ','
        << curvature_preview << ','
        << delta_raw << ','
        << (delta_rate_limited ? 1 : 0) << ','
        << delta_feedback << ','
        << delta_feedforward << ','
        << delta_cmd << ','
        << gain_y << ','
        << gain_psi << ','
        << sample.closest_idx << ','
        << sample.segment_idx << ','
        << sample.segment_t << ','
        << compute_time_ms << '\n';
    csv_file_.flush();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    const auto callback_start = std::chrono::steady_clock::now();
    if (points_.empty()) {
      return;
    }

    auto stamp = msg->header.stamp;
    if (stamp.sec == 0 && stamp.nanosec == 0) {
      stamp = get_clock()->now().to_msg();
    }
    const double stamp_sec =
        static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;

    const auto pose = resolve_pose(msg);
    if (!pose) {
      return;
    }
    const Point2 position = pose->first;
    const double yaw = pose->second;

    const double vx = msg->twist.twist.linear.x;
    const double vy = msg->twist.twist.linear.y;
    const double v_actual = std::hypot(vx, vy);
    const ReferenceSample sample = project_to_path(position);

    const Point2 normal = {-std::sin(sample.heading), std::cos(sample.heading)};
    const double raw_lateral_error = dot(subtract(position, sample.point), normal);
    const double raw_heading_error = pnc_wrap_angle(yaw - sample.heading);
    const auto filtered = filter_control_errors(raw_lateral_error, raw_heading_error);
    const double lateral_error = filtered.first;
    const double heading_error = filtered.second;

    double dt = control_dt_;
    if (previous_stamp_sec_) {
      const double measured_dt = stamp_sec - *previous_stamp_sec_;
      if (measured_dt >= 1e-4 && measured_dt <= 0.5) {
        dt = measured_dt;
      }
    }

    const double lqr_model_speed =
        std::max({v_actual, current_speed_cmd_, lqr_min_model_speed_});
    const GainEntry lqr_params = interpolate_gain(lqr_model_speed);
    const PncLqrResult lqr = pnc_compute_lqr_steering(
        lateral_error,
        heading_error,
        sample.curvature,
        lqr_model_speed,
        wheelbase_,
        dt,
        lqr_min_model_speed_,
        lqr_params.q_lateral,
        lqr_params.q_heading,
        lqr_params.r_steering,
        lqr_params.feedforward_gain);
    double delta_cmd =
        std::clamp(lqr.steering, -max_steering_angle_, max_steering_angle_);
    const auto steering_limit = apply_steering_rate_limit(delta_cmd, dt);
    delta_cmd = steering_limit.first;
    const bool delta_rate_limited = steering_limit.second;

    const double curvature_preview = preview_curvature(sample);
    const double desired_speed = enable_curvature_speed_limit_
        ? pnc_compute_curvature_limited_speed(
              target_speed_,
              curvature_preview,
              delta_cmd,
              wheelbase_,
              max_lateral_accel_,
              min_speed_)
        : target_speed_;
    double v_cmd = apply_speed_ramp(desired_speed, stamp_sec);

    if (open_loop_finished(position)) {
      if (!open_loop_finished_) {
        RCLCPP_INFO(get_logger(), "LQR open-loop endpoint reached; stopping.");
      }
      open_loop_finished_ = true;
      delta_cmd = 0.0;
      v_cmd = 0.0;
      current_speed_cmd_ = 0.0;
      previous_delta_cmd_ = 0.0;
    }

    publish_drive(stamp, delta_cmd, v_cmd);
    append_tracked_pose(position, yaw, stamp);
    const double compute_time_ms =
        std::chrono::duration<double, std::milli>(
            std::chrono::steady_clock::now() - callback_start)
            .count();
    write_tracking_log(
        stamp_sec,
        position,
        yaw,
        v_actual,
        v_cmd,
        raw_lateral_error,
        raw_heading_error,
        lateral_error,
        heading_error,
        sample,
        curvature_preview,
        lqr.steering,
        delta_rate_limited,
        lqr.feedback,
        lqr.feedforward,
        delta_cmd,
        lqr.gain_y,
        lqr.gain_psi,
        compute_time_ms);
    previous_stamp_sec_ = stamp_sec;
  }

  std::string path_topic_;
  std::string odom_topic_;
  std::string drive_topic_;
  std::string tracked_frame_;
  bool use_tf_pose_ = true;
  std::string vehicle_frame_;
  double tf_lookup_timeout_sec_ = 0.02;
  bool path_closed_loop_ = true;
  double open_loop_finish_distance_ = 0.25;
  double wheelbase_ = 0.3302;
  double target_speed_ = 0.60;
  double min_speed_ = 0.25;
  double max_steering_angle_ = 0.61;
  double max_lateral_accel_ = 3.0;
  bool enable_curvature_speed_limit_ = true;
  double curvature_speed_lookahead_m_ = 1.0;
  bool enable_speed_ramp_ = true;
  double max_accel_ = 0.8;
  double max_decel_ = 1.5;
  bool enable_steering_rate_limit_ = true;
  double max_steering_rate_ = 2.0;
  bool enable_error_filter_ = false;
  double error_filter_alpha_y_ = 0.30;
  double error_filter_alpha_psi_ = 0.25;
  double control_dt_ = 0.05;
  double lqr_min_model_speed_ = 0.25;
  double lqr_lookahead_distance_m_ = 1.5;
  double lqr_q_lateral_ = 3.0;
  double lqr_q_heading_ = 1.2;
  double lqr_r_steering_ = 8.0;
  double lqr_feedforward_gain_ = 1.0;
  std::vector<GainEntry> gain_table_;
  bool enable_tracking_csv_log_ = true;
  std::string tracking_log_path_;
  std::string tracked_path_topic_ = "/tracked_path_lqr";
  int64_t max_tracked_path_points_ = 2000;

  std::vector<Point2> points_;
  std::vector<double> headings_;
  std::vector<double> curvatures_;
  std::vector<double> segment_lengths_;
  std::string path_frame_id_ = "map";
  std::string path_signature_;
  std::optional<double> previous_stamp_sec_;
  double previous_delta_cmd_ = 0.0;
  double current_speed_cmd_ = 0.0;
  bool open_loop_finished_ = false;
  std::optional<double> filtered_lateral_error_;
  std::optional<double> filtered_heading_error_;
  std::ofstream csv_file_;
  nav_msgs::msg::Path tracked_path_msg_;

  tf2_ros::Buffer tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
  rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr tracked_path_pub_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LqrControllerNode>());
  rclcpp::shutdown();
  return 0;
}
