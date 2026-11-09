#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/empty.hpp>

namespace
{
using Matrix3 = Eigen::Matrix3d;
using Vector3 = Eigen::Vector3d;

bool finite(const Vector3 & v)
{
  return v.array().isFinite().all();
}

double finite_or(double value, double fallback)
{
  return std::isfinite(value) && value >= 0.0 ? value : fallback;
}
}  // namespace

class Fastlio2Px4OdomBridge final : public rclcpp::Node
{
public:
  Fastlio2Px4OdomBridge()
  : Node("fastlio2_px4_odom_bridge")
  {
    declare_parameter("input_topic", "/fastlio2/lio_odom");
    declare_parameter("output_topic", "/fmu/in/vehicle_visual_odometry");
    declare_parameter("px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("emergency_stop_topic", "/planning/emergency_stop");
    declare_parameter("input_frame", "lidar");
    declare_parameter("reject_frame_mismatch", true);
    declare_parameter("position_variance_floor", 0.01);
    declare_parameter("orientation_variance_floor", 0.01);
    declare_parameter("velocity_variance_floor", 0.04);
    declare_parameter("max_linear_speed_mps", 8.0);
    declare_parameter("max_horizontal_position_m", 50.0);
    declare_parameter("max_vertical_position_m", 8.0);
    declare_parameter("max_position_step_slack_m", 0.35);
    declare_parameter("max_position_velocity_residual_m", 0.25);
    // PGO may apply a bounded loop-closure correction to the absolute pose.
    // Treat such a correction as an estimator reset (VehicleOdometry
    // reset_counter) instead of converting it into an emergency stop.
    declare_parameter("max_pose_reset_jump_m", 1.50);
    declare_parameter("max_sample_interval_sec", 0.75);
    declare_parameter("max_sample_age_sec", 0.75);
    declare_parameter("max_attitude_step_rad", 1.3);
    declare_parameter("transform_translation", std::vector<double>{0.0, 0.0, 0.0});
    declare_parameter("world_rotation_row_major", std::vector<double>{
      0.0, 1.0, 0.0,
      1.0, 0.0, 0.0,
      0.0, 0.0, -1.0});
    // FAST-LIO consumes the Gazebo IMU after the MID-360 plugin removes the
    // sensor mount yaw, leaving the X500 base_link convention (FLU). PX4 uses
    // FRD, so FLU -> FRD is a pi rotation about X.
    declare_parameter("body_to_frd_row_major", std::vector<double>{
      1.0, 0.0, 0.0,
      0.0, -1.0, 0.0,
      0.0, 0.0, -1.0});
    declare_parameter("align_to_px4_local_origin", true);

    input_frame_ = get_parameter("input_frame").as_string();
    reject_frame_mismatch_ = get_parameter("reject_frame_mismatch").as_bool();
    position_floor_ = get_parameter("position_variance_floor").as_double();
    orientation_floor_ = get_parameter("orientation_variance_floor").as_double();
    velocity_floor_ = get_parameter("velocity_variance_floor").as_double();
    max_linear_speed_mps_ = get_parameter("max_linear_speed_mps").as_double();
    max_horizontal_position_m_ = get_parameter("max_horizontal_position_m").as_double();
    max_vertical_position_m_ = get_parameter("max_vertical_position_m").as_double();
    max_position_step_slack_m_ = get_parameter("max_position_step_slack_m").as_double();
    max_position_velocity_residual_m_ =
      get_parameter("max_position_velocity_residual_m").as_double();
    max_pose_reset_jump_m_ = get_parameter("max_pose_reset_jump_m").as_double();
    max_sample_interval_sec_ = get_parameter("max_sample_interval_sec").as_double();
    max_sample_age_sec_ = get_parameter("max_sample_age_sec").as_double();
    max_attitude_step_rad_ = get_parameter("max_attitude_step_rad").as_double();
    align_to_px4_local_origin_ = get_parameter("align_to_px4_local_origin").as_bool();
    translation_ = vector3_parameter("transform_translation");
    world_rotation_ = rotation_parameter("world_rotation_row_major");
    body_to_frd_ = rotation_parameter("body_to_frd_row_major");
    if (!std::isfinite(position_floor_) || !std::isfinite(orientation_floor_) ||
      !std::isfinite(velocity_floor_) || position_floor_ < 0.0 ||
      orientation_floor_ < 0.0 || velocity_floor_ < 0.0 ||
      !std::isfinite(max_linear_speed_mps_) || max_linear_speed_mps_ <= 0.0 ||
      !std::isfinite(max_horizontal_position_m_) || max_horizontal_position_m_ <= 0.0 ||
      !std::isfinite(max_vertical_position_m_) || max_vertical_position_m_ <= 0.0 ||
      !std::isfinite(max_position_step_slack_m_) || max_position_step_slack_m_ < 0.0 ||
      !std::isfinite(max_position_velocity_residual_m_) ||
      max_position_velocity_residual_m_ < 0.0 ||
      !std::isfinite(max_pose_reset_jump_m_) || max_pose_reset_jump_m_ <= 0.0 ||
      !std::isfinite(max_sample_interval_sec_) || max_sample_interval_sec_ <= 0.0 ||
      !std::isfinite(max_sample_age_sec_) || max_sample_age_sec_ <= 0.0 ||
      !std::isfinite(max_attitude_step_rad_) || max_attitude_step_rad_ <= 0.0 ||
      max_attitude_step_rad_ > 3.141592653589793)
    {
      throw std::runtime_error("variance floors must be finite and non-negative");
    }

    const auto qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      get_parameter("input_topic").as_string(), qos,
      std::bind(&Fastlio2Px4OdomBridge::odom_callback, this, std::placeholders::_1));
    odom_pub_ = create_publisher<px4_msgs::msg::VehicleOdometry>(
      get_parameter("output_topic").as_string(), qos);
    emergency_pub_ = create_publisher<std_msgs::msg::Empty>(
      get_parameter("emergency_stop_topic").as_string(), rclcpp::QoS(10));
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      get_parameter("px4_local_position_topic").as_string(), qos,
      [this](const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) {
        px4_timestamp_us_ = msg->timestamp;
        px4_local_position_ = Vector3(msg->x, msg->y, msg->z);
        // During EKF startup xy_valid can be false even though PX4 has
        // already established its local origin.  The first finite sample is
        // sufficient for frame alignment; the controller separately requires
        // all validity flags before arming.
        px4_local_position_valid_ = msg->timestamp > 0 && px4_local_position_.allFinite();
      });
    RCLCPP_INFO(get_logger(), "FAST-LIO odometry bridge: %s -> %s",
      get_parameter("input_topic").as_string().c_str(),
      get_parameter("output_topic").as_string().c_str());
  }

private:
  void update_clock_mapping(int64_t ros_now_ns, uint64_t px4_timestamp_us)
  {
    const double ros_now_us = static_cast<double>(ros_now_ns) / 1000.0;
    if (last_sync_ros_us_ && last_sync_px4_timestamp_us_) {
      const double delta_ros_us = ros_now_us - *last_sync_ros_us_;
      const double delta_px4_us = static_cast<double>(px4_timestamp_us) -
        static_cast<double>(*last_sync_px4_timestamp_us_);
      if (delta_ros_us >= 1000000.0 && delta_px4_us > 0.0) {
        const double observed_rate = delta_px4_us / delta_ros_us;
        if (std::isfinite(observed_rate) && observed_rate >= 0.1 && observed_rate <= 10.0) {
          px4_ros_clock_rate_ = 0.85 * px4_ros_clock_rate_ + 0.15 * observed_rate;
        }
        last_sync_ros_us_ = ros_now_us;
        last_sync_px4_timestamp_us_ = px4_timestamp_us;
      }
    } else {
      last_sync_ros_us_ = ros_now_us;
      last_sync_px4_timestamp_us_ = px4_timestamp_us;
    }

    timestamp_offset_us_ = static_cast<double>(px4_timestamp_us) - px4_ros_clock_rate_ * ros_now_us;
    if (!clock_mapping_logged_) {
      RCLCPP_INFO(get_logger(), "Started FAST-LIO to PX4 clock mapping: rate %.7f, offset %.3f ms",
        px4_ros_clock_rate_, *timestamp_offset_us_ / 1000.0);
      clock_mapping_logged_ = true;
    }
  }

  Vector3 vector3_parameter(const std::string & name)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain 3 values");
    }
    return Vector3(values[0], values[1], values[2]);
  }

  Matrix3 rotation_parameter(const std::string & name)
  {
    const auto values = get_parameter(name).as_double_array();
    if (values.size() != 9) {
      throw std::runtime_error(name + " must contain 9 values");
    }
    Matrix3 result;
    for (int row = 0; row < 3; ++row) {
      for (int col = 0; col < 3; ++col) {
        result(row, col) = values[static_cast<size_t>(row * 3 + col)];
      }
    }
    if (!result.allFinite() || (result * result.transpose() - Matrix3::Identity()).norm() > 1e-4 ||
      std::abs(result.determinant() - 1.0) > 1e-4)
    {
      throw std::runtime_error(name + " must be a proper rotation matrix");
    }
    return result;
  }

  static float variance(const std::array<double, 36> & covariance, size_t index, double floor)
  {
    return static_cast<float>(finite_or(covariance[index], floor) < floor ?
      floor : finite_or(covariance[index], floor));
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (fault_latched_) {
      return;
    }
    if (reject_frame_mismatch_ && msg->header.frame_id != input_frame_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Rejecting odometry frame '%s'; expected '%s'", msg->header.frame_id.c_str(), input_frame_.c_str());
      return;
    }
    const auto & p = msg->pose.pose.position;
    const auto & q_msg = msg->pose.pose.orientation;
    const Vector3 position_in(p.x, p.y, p.z);
    const Vector3 velocity_body(
      msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
    const Vector3 angular_body(
      msg->twist.twist.angular.x, msg->twist.twist.angular.y, msg->twist.twist.angular.z);
    Eigen::Quaterniond q_in(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
    if (!finite(position_in) || !finite(velocity_body) || !finite(angular_body) ||
      !std::isfinite(q_in.norm()) || q_in.norm() < 1e-6)
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Rejecting non-finite FAST-LIO odometry");
      return;
    }
    q_in.normalize();

    // q_in maps FAST-LIO child/body axes into the lidar world. The output
    // quaternion must map PX4 FRD body axes into PX4 NED world axes.
    const Matrix3 body_to_lidar = q_in.toRotationMatrix();
    const Matrix3 frd_to_body = body_to_frd_.transpose();
    const Matrix3 ned_from_frd = world_rotation_ * body_to_lidar * frd_to_body;
    const Vector3 raw_position_ned = world_rotation_ * position_in;
    if (!origin_aligned_) {
      if (align_to_px4_local_origin_ && !px4_local_position_valid_) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "Waiting for valid PX4 local position before aligning external-vision origin");
        return;
      }
      if (align_to_px4_local_origin_ && px4_local_position_valid_) {
        translation_ = px4_local_position_ - raw_position_ned;
      } else {
        translation_ = -raw_position_ned;
      }
      origin_aligned_ = true;
      RCLCPP_INFO(get_logger(), "Aligned FAST-LIO origin: translation NED [%.3f %.3f %.3f]",
        translation_.x(), translation_.y(), translation_.z());
    }
    const Vector3 position_ned = raw_position_ned + translation_;
    Vector3 velocity_ned = world_rotation_ * body_to_lidar * velocity_body;
    const Vector3 angular_frd = body_to_frd_ * angular_body;
    const Eigen::Quaterniond q_out(ned_from_frd);
    if (!finite(position_ned) || !finite(velocity_ned) || !finite(angular_frd) ||
      !std::isfinite(q_out.norm()) || q_out.norm() < 1e-6)
    {
      latch_fault("transformed odometry contains invalid values");
      return;
    }
    if (std::hypot(position_ned.x(), position_ned.y()) > max_horizontal_position_m_ ||
      std::abs(position_ned.z()) > max_vertical_position_m_)
    {
      latch_fault("odometry position exceeded configured local-flight bounds");
      return;
    }
    const int64_t stamp_ns = static_cast<int64_t>(msg->header.stamp.sec) * 1000000000LL +
      msg->header.stamp.nanosec;
    if (stamp_ns <= 0) {
      latch_fault("odometry timestamp is zero or negative");
      return;
    }
    const int64_t ros_now_ns = get_clock()->now().nanoseconds();
    if (ros_now_ns <= 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for ROS simulation clock before timestamping external odometry");
      return;
    }
    const double sample_age_sec = static_cast<double>(ros_now_ns - stamp_ns) / 1e9;
    if (sample_age_sec > max_sample_age_sec_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Skipping stale FAST-LIO sample: age %.3f s exceeds %.3f s",
        sample_age_sec, max_sample_age_sec_);
      return;
    }
    if (sample_age_sec < -0.05) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Skipping FAST-LIO sample %.3f s ahead of ROS simulation clock", -sample_age_sec);
      return;
    }
    const double speed = velocity_ned.norm();
    if (!std::isfinite(speed) || speed > max_linear_speed_mps_) {
      latch_fault("odometry velocity exceeded max_linear_speed_mps");
      return;
    }
    bool attitude_jump = false;
    bool pose_reset = false;
    if (last_stamp_ns_) {
      const double dt = static_cast<double>(stamp_ns - *last_stamp_ns_) / 1e9;
      if (dt <= 0.0) {
        latch_fault("odometry timestamp repeated or moved backwards");
        return;
      }
      const double allowed_step = max_position_step_slack_m_ + max_linear_speed_mps_ * dt;
      const double position_step = (position_ned - *last_position_ned_).norm();
      if (position_step > allowed_step) {
        if (position_step <= max_pose_reset_jump_m_) {
          pose_reset = true;
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
            "Accepting bounded PGO pose reset: step %.3f m (limit %.3f m)",
            position_step, max_pose_reset_jump_m_);
        } else {
          latch_fault("position jumped farther than the configured reset bound allows");
          return;
        }
      }
      attitude_jump = q_out.angularDistance(*last_orientation_) > max_attitude_step_rad_;
      if (dt <= max_sample_interval_sec_ && !attitude_jump && !pose_reset) {
        const Vector3 expected_step = 0.5 * (*last_velocity_ned_ + velocity_ned) * dt;
        const double residual = ((position_ned - *last_position_ned_) - expected_step).norm();
        if (residual > max_position_velocity_residual_m_) {
          if (residual <= max_pose_reset_jump_m_) {
            pose_reset = true;
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
              "Accepting bounded PGO position/velocity reset: residual %.3f m (limit %.3f m)",
              residual, max_pose_reset_jump_m_);
          } else {
            latch_fault("position and velocity estimates exceeded the configured reset bound");
            return;
          }
        }
      } else {
        if (dt > max_sample_interval_sec_) {
          RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
            "FAST-LIO sample gap %.3f s; accepting valid absolute pose and restarting continuity checks",
            dt);
        }
      }
    }
    if (attitude_jump || pose_reset) {
      velocity_ned.setZero();
      if (pose_reset) {
        ++reset_counter_;
      }
      if (pose_reset) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Publishing bounded FAST-LIO pose reset %u with zero velocity",
          static_cast<unsigned>(reset_counter_));
      } else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "FAST-LIO attitude jumped; publishing position while suppressing velocity for this sample");
      }
    }
    if (px4_timestamp_us_ == 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Waiting for PX4 local-position timestamp before publishing external odometry");
      return;
    }

    px4_msgs::msg::VehicleOdometry out;
    // FAST-LIO stamps use ROS simulation time and PX4 uses its own clock.
    // Continuously estimate their rate and offset so clock drift does not make
    // recent sensor samples appear seconds old to EKF2.
    update_clock_mapping(ros_now_ns, px4_timestamp_us_);
    const uint64_t input_stamp_us = static_cast<uint64_t>(stamp_ns / 1000LL);
    const double mapped_sample_double = px4_ros_clock_rate_ * static_cast<double>(input_stamp_us) +
      *timestamp_offset_us_;
    if (!std::isfinite(mapped_sample_double) ||
      mapped_sample_double > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
      mapped_sample_double < static_cast<double>(std::numeric_limits<int64_t>::min()))
    {
      latch_fault("mapped odometry timestamp is outside representable range");
      return;
    }
    const int64_t mapped_sample = static_cast<int64_t>(std::llround(mapped_sample_double));
    if (mapped_sample <= 0) {
      latch_fault("mapped odometry timestamp is zero or negative");
      return;
    }
    constexpr int64_t kFutureToleranceUs = 50000;
    if (mapped_sample > static_cast<int64_t>(px4_timestamp_us_) + kFutureToleranceUs) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Skipping FAST-LIO sample %.3f ms ahead of PX4 clock",
        static_cast<double>(mapped_sample - static_cast<int64_t>(px4_timestamp_us_)) / 1000.0);
      return;
    }
    if (last_mapped_sample_us_ && mapped_sample <= *last_mapped_sample_us_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "Skipping non-monotonic mapped FAST-LIO timestamp");
      return;
    }
    out.timestamp = px4_timestamp_us_;
    out.timestamp_sample = static_cast<uint64_t>(mapped_sample);
    out.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;
    out.position = {static_cast<float>(position_ned.x()), static_cast<float>(position_ned.y()),
      static_cast<float>(position_ned.z())};
    out.q = {static_cast<float>(q_out.w()), static_cast<float>(q_out.x()),
      static_cast<float>(q_out.y()), static_cast<float>(q_out.z())};
    out.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    out.velocity = {static_cast<float>(velocity_ned.x()), static_cast<float>(velocity_ned.y()),
      static_cast<float>(velocity_ned.z())};
    out.angular_velocity = {static_cast<float>(angular_frd.x()), static_cast<float>(angular_frd.y()),
      static_cast<float>(angular_frd.z())};
    out.position_variance = {
      variance(msg->pose.covariance, 0, position_floor_),
      variance(msg->pose.covariance, 7, position_floor_),
      variance(msg->pose.covariance, 14, position_floor_)};
    out.orientation_variance = {
      variance(msg->pose.covariance, 21, orientation_floor_),
      variance(msg->pose.covariance, 28, orientation_floor_),
      variance(msg->pose.covariance, 35, orientation_floor_)};
    out.velocity_variance = (attitude_jump || pose_reset) ? std::array<float, 3>{100.0F, 100.0F, 100.0F} :
      std::array<float, 3>{
      variance(msg->twist.covariance, 0, velocity_floor_),
      variance(msg->twist.covariance, 7, velocity_floor_),
      variance(msg->twist.covariance, 14, velocity_floor_)};
    out.reset_counter = reset_counter_;
    out.quality = 100;
    odom_pub_->publish(out);
    last_stamp_ns_ = stamp_ns;
    last_mapped_sample_us_ = mapped_sample;
    last_position_ned_ = position_ned;
    last_velocity_ned_ = velocity_ned;
    last_orientation_ = q_out;
  }

  void latch_fault(const std::string & reason)
  {
    fault_latched_ = true;
    emergency_pub_->publish(std_msgs::msg::Empty{});
    RCLCPP_ERROR(get_logger(),
      "External odometry disabled and emergency stop requested: %s. Restart this bridge only after FAST-LIO is healthy.",
      reason.c_str());
  }

  std::string input_frame_;
  bool reject_frame_mismatch_{true};
  bool align_to_px4_local_origin_{true};
  double position_floor_{0.01};
  double orientation_floor_{0.01};
  double velocity_floor_{0.04};
  double max_linear_speed_mps_{8.0};
  double max_horizontal_position_m_{50.0};
  double max_vertical_position_m_{8.0};
  double max_position_step_slack_m_{0.35};
  double max_position_velocity_residual_m_{0.25};
  double max_pose_reset_jump_m_{1.50};
  double max_sample_interval_sec_{0.75};
  double max_sample_age_sec_{0.75};
  double max_attitude_step_rad_{1.3};
  Vector3 translation_{};
  Matrix3 world_rotation_{Matrix3::Identity()};
  Matrix3 body_to_frd_{Matrix3::Identity()};
  uint8_t reset_counter_{0};
  bool fault_latched_{false};
  bool origin_aligned_{false};
  bool px4_local_position_valid_{false};
  Vector3 px4_local_position_{};
  std::optional<int64_t> last_stamp_ns_;
  std::optional<int64_t> last_mapped_sample_us_;
  std::optional<Vector3> last_position_ned_;
  std::optional<Vector3> last_velocity_ned_;
  std::optional<Eigen::Quaterniond> last_orientation_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr emergency_pub_;
  uint64_t px4_timestamp_us_{0};
  double px4_ros_clock_rate_{1.0};
  std::optional<double> timestamp_offset_us_;
  std::optional<double> last_sync_ros_us_;
  std::optional<uint64_t> last_sync_px4_timestamp_us_;
  bool clock_mapping_logged_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Fastlio2Px4OdomBridge>());
  rclcpp::shutdown();
  return 0;
}
