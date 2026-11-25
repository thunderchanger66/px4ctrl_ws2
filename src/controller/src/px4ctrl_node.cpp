#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "controller/arm_cdm_parameters.hpp"
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"
#include "px4_msgs/msg/vehicle_attitude_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_land_detected.hpp"
#include "px4_msgs/msg/vehicle_local_position.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "controller/msg/control_debug.hpp"
#include "controller/msg/desired_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

namespace
{
using DesiredStateMsg = controller::msg::DesiredState;
using ControlDebug = controller::msg::ControlDebug;
using Vector3 = Eigen::Vector3d;
using Matrix3 = Eigen::Matrix3d;

struct DesiredState //参考的内部表示
{
  Vector3 position{Vector3::Zero()};
  Vector3 velocity{Vector3::Zero()};
  Vector3 acceleration{Vector3::Zero()};
  double yaw{0.0};
  double yaw_rate{0.0};
};

struct ControlOutput //控制输出的内部表示
{
  Eigen::Quaterniond attitude{Eigen::Quaterniond::Identity()};
  double thrust{0.5};
  double yaw_rate{0.0};
  double tilt_deg{0.0};
  uint8_t saturation_flags{0};
  Vector3 position_error{Vector3::Zero()};
  Vector3 velocity_error{Vector3::Zero()};
  Vector3 velocity_reference{Vector3::Zero()};
  Vector3 acceleration_requested{Vector3::Zero()};
  Vector3 acceleration_feasible{Vector3::Zero()};
};

Vector3 limit_body_z_tilt(Vector3 body_z, double max_tilt_rad) //倾角限制
{
  constexpr double kEpsilon = 1e-12;
  const Vector3 world_z(0.0, 0.0, 1.0);
  const double norm = body_z.norm();
  if (!std::isfinite(norm) || norm < kEpsilon) return world_z;
  body_z /= norm;
  const double dot = std::clamp(body_z.dot(world_z), -1.0, 1.0);
  const double angle = std::min(std::acos(dot), max_tilt_rad);
  Vector3 rejection = body_z - dot * world_z;
  const double rejection_norm = rejection.norm();
  if (rejection_norm < kEpsilon) {
    rejection = Vector3(1.0, 0.0, 0.0);
  } else {
    rejection /= rejection_norm;
  }
  return std::cos(angle) * world_z + std::sin(angle) * rejection;
}

Eigen::Quaterniond body_z_and_yaw_to_attitude(const Vector3 & body_z, double yaw) //姿态构造
{
  const Vector3 y_c(-std::sin(yaw), std::cos(yaw), 0.0);
  Vector3 body_x = y_c.cross(body_z);
  if (body_z.z() < 0.0) body_x = -body_x;
  if (body_x.squaredNorm() < 1e-12) {
    body_x = Vector3(0.0, 0.0, 1.0);
  } else {
    body_x.normalize();
  }
  Vector3 body_y = body_z.cross(body_x);
  if (body_y.squaredNorm() < 1e-12) body_y = Vector3(0.0, 1.0, 0.0);
  else body_y.normalize();
  Matrix3 rotation;
  rotation.col(0) = body_x;
  rotation.col(1) = body_y;
  rotation.col(2) = body_z;
  return Eigen::Quaterniond(rotation).normalized();
}

enum class ControlState //状态
{
  waiting_for_command,
  takeoff,
  hover,
  tracking,
  safety_fault
};

bool finite(const Vector3 & value)
{
  return value.allFinite();
}

class Px4Ctrl final : public rclcpp::Node
{
public:
  Px4Ctrl()
  : Node("px4ctrl") //构造函数：参数声明/校验/订阅/发布/定时器
  {
    for (const auto & override_param : get_node_parameters_interface()->get_parameter_overrides()) {
      const std::string & name = override_param.first;
      if (name == "planner_frame" || name == "odom_topic" || name == "state_timeout_sec" ||
        name == "align_to_px4_local_origin" ||
        name == "frame_alignment_velocity_tolerance_mps" ||
        name == "transform_rotation_row_major")
      {
        throw std::runtime_error(
          "parameter '" + name + "' was removed: the reference must already be expressed in "
          "PX4 local NED. Use 'reference_frame' for the frame id and remove '" + name + "'.");
      }
    }

    declare_parameter("reference_frame", "px4_local_ned");
    declare_parameter("desired_state_topic", "/px4ctrl/desired_state");
    declare_parameter("debug_topic", "/px4ctrl/debug");
    declare_parameter("entry_mode", "auto_takeoff");
    declare_parameter("takeover_velocity_tolerance_mps", 0.20);
    declare_parameter("require_takeover_permission", false);
    declare_parameter("takeover_permission_topic", "/eight/takeover_allowed");
    declare_parameter("px4_status_topic", "/fmu/out/vehicle_status_v4");
    declare_parameter("px4_attitude_topic", "/fmu/out/vehicle_attitude");
    declare_parameter("px4_local_position_topic", "/fmu/out/vehicle_local_position_v1");
    declare_parameter("px4_land_detected_topic", "/fmu/out/vehicle_land_detected");
    declare_parameter("offboard_control_mode_topic", "/fmu/in/offboard_control_mode");
    declare_parameter("attitude_setpoint_topic", "/fmu/in/vehicle_attitude_setpoint_v1");
    declare_parameter("vehicle_command_topic", "/fmu/in/vehicle_command");
    declare_parameter("status_topic", "/px4ctrl/status");
    declare_parameter("manual_yaw_topic", "/px4ctrl/manual_yaw");
    declare_parameter("clear_manual_yaw_topic", "/px4ctrl/clear_manual_yaw");
    declare_parameter("publish_rate_hz", 100.0);
    declare_parameter("command_timeout_sec", 0.50);
    declare_parameter("prestream_seconds", 1.0);
    declare_parameter("start_offboard_on_first_command", true);
    declare_parameter("arm_on_first_command", true);
    declare_parameter("gravity_mps2", 9.81);
    declare_parameter("hover_thrust", 0.864);
    declare_parameter("min_thrust", 0.08);
    declare_parameter("max_thrust", 0.95);
    declare_parameter("max_tilt_deg", 30.0);
    declare_parameter("max_acceleration_mps2", 4.0);
    declare_parameter("local_position_timeout_sec", 0.50);
    declare_parameter("land_detected_timeout_sec", 2.0);
    declare_parameter("px4_status_timeout_sec", 1.50);
    declare_parameter("local_position_fault_delay_sec", 1.00);
    declare_parameter("max_ekf_reset_delta_m", 1.0);
    declare_parameter("max_ekf_reset_heading_rad", 0.35);
    declare_parameter("require_local_position_valid", true);
    declare_parameter("max_local_position_m", 100.0);
    declare_parameter("max_local_velocity_mps", 15.0);
    declare_parameter("land_on_safety_fault", true);
    declare_parameter("auto_takeoff_altitude_m", 0.50);
    declare_parameter("minimum_takeoff_altitude_m", 0.25);
    declare_parameter("minimum_tracking_altitude_m", 0.25);
    declare_parameter("takeoff_duration_sec", 3.0);
    declare_parameter("takeoff_tolerance_m", 0.08);
    declare_parameter("hover_settle_sec", 0.80);
    declare_parameter("hover_velocity_tolerance_mps", 0.15);
    declare_parameter("tracking_handoff_sec", 0.75);
    declare_parameter("command_blend_sec", 0.25);
    declare_parameter("max_command_position_jump_m", 0.35);
    declare_parameter("max_command_velocity_jump_mps", 1.5);
    declare_parameter("max_yaw_rate_radps", 0.6);
    declare_parameter("max_yaw_acceleration_radps2", 0.8);
    declare_parameter("kp", std::vector<double>{1.5, 1.5, 1.8});
    declare_parameter("kv", std::vector<double>{1.2, 1.2, 1.4});
    declare_parameter("ki", std::vector<double>{0.10, 0.10, 0.15});
    declare_parameter("integral_limit_mps2", std::vector<double>{0.40, 0.40, 0.50});
    declare_parameter("integral_enable_velocity_error_mps", 0.60);
    declare_parameter("enable_cdm_feedforward", false);
    declare_parameter("cdm_joint_state_topic", "/joint_states");
    declare_parameter("cdm_odometry_topic", "/fmu/out/vehicle_odometry");
    declare_parameter("cdm_state_timeout_sec", 0.20);
    declare_parameter("cdm_max_acceleration_mps2", 1.0);

    enable_cdm_feedforward_ = get_parameter("enable_cdm_feedforward").as_bool();
    cdm_state_timeout_sec_ = get_parameter("cdm_state_timeout_sec").as_double();
    cdm_max_acceleration_mps2_ = get_parameter("cdm_max_acceleration_mps2").as_double();
    if (!std::isfinite(cdm_state_timeout_sec_) || cdm_state_timeout_sec_ <= 0.0 ||
      !std::isfinite(cdm_max_acceleration_mps2_) || cdm_max_acceleration_mps2_ <= 0.0)
    {
      throw std::runtime_error("CDM timeout and acceleration limit must be positive");
    }

    if (enable_cdm_feedforward_) {
      initialize_cdm_model();
    }

    reference_frame_ = get_parameter("reference_frame").as_string();
    entry_mode_ = get_parameter("entry_mode").as_string();
    takeover_velocity_tolerance_mps_ = get_parameter("takeover_velocity_tolerance_mps").as_double();
    require_takeover_permission_ = get_parameter("require_takeover_permission").as_bool();
    command_timeout_sec_ = get_parameter("command_timeout_sec").as_double();
    prestream_seconds_ = get_parameter("prestream_seconds").as_double();
    start_offboard_ = get_parameter("start_offboard_on_first_command").as_bool();
    arm_on_first_ = get_parameter("arm_on_first_command").as_bool();
    gravity_ = get_parameter("gravity_mps2").as_double();
    hover_thrust_ = get_parameter("hover_thrust").as_double();
    min_thrust_ = get_parameter("min_thrust").as_double();
    max_thrust_ = get_parameter("max_thrust").as_double();
    max_tilt_rad_ = get_parameter("max_tilt_deg").as_double() * M_PI / 180.0;
    max_acceleration_ = get_parameter("max_acceleration_mps2").as_double();
    local_position_timeout_sec_ = get_parameter("local_position_timeout_sec").as_double();
    land_detected_timeout_sec_ = get_parameter("land_detected_timeout_sec").as_double();
    px4_status_timeout_sec_ = get_parameter("px4_status_timeout_sec").as_double();
    local_position_fault_delay_sec_ = get_parameter("local_position_fault_delay_sec").as_double();
    max_ekf_reset_delta_m_ = get_parameter("max_ekf_reset_delta_m").as_double();
    max_ekf_reset_heading_rad_ = get_parameter("max_ekf_reset_heading_rad").as_double();
    require_local_position_valid_ = get_parameter("require_local_position_valid").as_bool();
    max_local_position_m_ = get_parameter("max_local_position_m").as_double();
    max_local_velocity_mps_ = get_parameter("max_local_velocity_mps").as_double();
    land_on_safety_fault_ = get_parameter("land_on_safety_fault").as_bool();
    auto_takeoff_altitude_m_ = get_parameter("auto_takeoff_altitude_m").as_double();
    minimum_takeoff_altitude_m_ = get_parameter("minimum_takeoff_altitude_m").as_double();
    minimum_tracking_altitude_m_ = get_parameter("minimum_tracking_altitude_m").as_double();
    takeoff_duration_sec_ = get_parameter("takeoff_duration_sec").as_double();
    takeoff_tolerance_m_ = get_parameter("takeoff_tolerance_m").as_double();
    hover_settle_sec_ = get_parameter("hover_settle_sec").as_double();
    hover_velocity_tolerance_mps_ = get_parameter("hover_velocity_tolerance_mps").as_double();
    tracking_handoff_sec_ = get_parameter("tracking_handoff_sec").as_double();
    command_blend_sec_ = get_parameter("command_blend_sec").as_double();
    max_command_position_jump_m_ = get_parameter("max_command_position_jump_m").as_double();
    max_command_velocity_jump_mps_ = get_parameter("max_command_velocity_jump_mps").as_double();
    max_yaw_rate_radps_ = get_parameter("max_yaw_rate_radps").as_double();
    max_yaw_acceleration_radps2_ = get_parameter("max_yaw_acceleration_radps2").as_double();
    const auto kp = get_parameter("kp").as_double_array();
    const auto kv = get_parameter("kv").as_double_array();
    const auto ki = get_parameter("ki").as_double_array();
    const auto integral_limit = get_parameter("integral_limit_mps2").as_double_array();
    integral_enable_velocity_error_mps_ =
      get_parameter("integral_enable_velocity_error_mps").as_double();
    if (kp.size() != 3 || kv.size() != 3 || ki.size() != 3 || integral_limit.size() != 3 ||
        reference_frame_.empty() ||
        !std::isfinite(command_timeout_sec_) || command_timeout_sec_ <= 0.0 ||
        !std::isfinite(prestream_seconds_) || prestream_seconds_ < 0.0 ||
        !std::isfinite(gravity_) || gravity_ <= 0.0 || !std::isfinite(hover_thrust_) ||
        hover_thrust_ <= 0.0 || hover_thrust_ >= 1.0 ||
        !std::isfinite(min_thrust_) ||
        !std::isfinite(max_thrust_) || min_thrust_ < 0.0 || max_thrust_ > 1.0 ||
        min_thrust_ >= max_thrust_ || hover_thrust_ <= min_thrust_ ||
        hover_thrust_ >= max_thrust_ || !std::isfinite(max_tilt_rad_) || max_tilt_rad_ <= 0.0 ||
        max_tilt_rad_ >= M_PI_2 || !std::isfinite(max_acceleration_) || max_acceleration_ <= 0.0 ||
        !std::isfinite(local_position_timeout_sec_) || local_position_timeout_sec_ <= 0.0 ||
        !std::isfinite(land_detected_timeout_sec_) || land_detected_timeout_sec_ <= 0.0 ||
        !std::isfinite(px4_status_timeout_sec_) || px4_status_timeout_sec_ <= 0.0 ||
        !std::isfinite(takeover_velocity_tolerance_mps_) || takeover_velocity_tolerance_mps_ <= 0.0 ||
        (entry_mode_ != "auto_takeoff" && entry_mode_ != "airborne_takeover") ||
        !std::isfinite(local_position_fault_delay_sec_) || local_position_fault_delay_sec_ <= 0.0 ||
        !std::isfinite(max_ekf_reset_delta_m_) || max_ekf_reset_delta_m_ <= 0.0 ||
        !std::isfinite(max_ekf_reset_heading_rad_) || max_ekf_reset_heading_rad_ <= 0.0 ||
        !std::isfinite(max_local_position_m_) || max_local_position_m_ <= 0.0 ||
        !std::isfinite(max_local_velocity_mps_) || max_local_velocity_mps_ <= 0.0 ||
        !std::isfinite(auto_takeoff_altitude_m_) || auto_takeoff_altitude_m_ < 0.0 ||
        !std::isfinite(minimum_takeoff_altitude_m_) || minimum_takeoff_altitude_m_ < 0.0 ||
        minimum_takeoff_altitude_m_ > auto_takeoff_altitude_m_ ||
        !std::isfinite(minimum_tracking_altitude_m_) || minimum_tracking_altitude_m_ < 0.0 ||
        !std::isfinite(takeoff_duration_sec_) || takeoff_duration_sec_ <= 0.0 ||
        !std::isfinite(takeoff_tolerance_m_) || takeoff_tolerance_m_ <= 0.0 ||
        !std::isfinite(hover_settle_sec_) || hover_settle_sec_ <= 0.0 ||
        !std::isfinite(hover_velocity_tolerance_mps_) || hover_velocity_tolerance_mps_ <= 0.0 ||
        !std::isfinite(tracking_handoff_sec_) || tracking_handoff_sec_ < 0.0 ||
        !std::isfinite(command_blend_sec_) || command_blend_sec_ < 0.0 ||
        !std::isfinite(max_command_position_jump_m_) || max_command_position_jump_m_ <= 0.0 ||
        !std::isfinite(max_command_velocity_jump_mps_) || max_command_velocity_jump_mps_ <= 0.0 ||
        !std::isfinite(integral_enable_velocity_error_mps_) || integral_enable_velocity_error_mps_ <= 0.0 ||
        !std::isfinite(max_yaw_rate_radps_) || max_yaw_rate_radps_ <= 0.0 ||
        !std::isfinite(max_yaw_acceleration_radps2_) || max_yaw_acceleration_radps2_ <= 0.0)
    {
      throw std::runtime_error("invalid px4ctrl parameters");
    }
    for (size_t i = 0; i < 3; ++i) {
      kp_(static_cast<Eigen::Index>(i)) = kp[i];
      kv_(static_cast<Eigen::Index>(i)) = kv[i];
      ki_(static_cast<Eigen::Index>(i)) = ki[i];
      integral_limit_(static_cast<Eigen::Index>(i)) = integral_limit[i];
      if (!std::isfinite(kp[i]) || !std::isfinite(kv[i]) || !std::isfinite(ki[i]) ||
        !std::isfinite(integral_limit[i]) || kp[i] < 0.0 || kv[i] < 0.0 || ki[i] < 0.0 ||
        integral_limit[i] <= 0.0) {
        throw std::runtime_error("kp, kv, ki and integral limits must be finite and non-negative");
      }
    }
    const auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();
    command_sub_ = create_subscription<DesiredStateMsg>(
      get_parameter("desired_state_topic").as_string(), rclcpp::QoS(20),
      std::bind(&Px4Ctrl::command_callback, this, std::placeholders::_1));
    status_sub_ = create_subscription<px4_msgs::msg::VehicleStatus>(
      get_parameter("px4_status_topic").as_string(), px4_qos,
      std::bind(&Px4Ctrl::vehicle_status_callback, this, std::placeholders::_1));
    attitude_sub_ = create_subscription<px4_msgs::msg::VehicleAttitude>(
      get_parameter("px4_attitude_topic").as_string(), px4_qos,
      std::bind(&Px4Ctrl::attitude_callback, this, std::placeholders::_1));
    local_position_sub_ = create_subscription<px4_msgs::msg::VehicleLocalPosition>(
      get_parameter("px4_local_position_topic").as_string(), px4_qos,
      std::bind(&Px4Ctrl::local_position_callback, this, std::placeholders::_1));
    if (enable_cdm_feedforward_) {
      odometry_sub_ = create_subscription<px4_msgs::msg::VehicleOdometry>(
        get_parameter("cdm_odometry_topic").as_string(), px4_qos,
        std::bind(&Px4Ctrl::cdm_odometry_callback, this, std::placeholders::_1));
      joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
        get_parameter("cdm_joint_state_topic").as_string(),
        rclcpp::QoS(rclcpp::SensorDataQoS()),
        std::bind(&Px4Ctrl::joint_state_callback, this, std::placeholders::_1));
    }
    land_detected_sub_ = create_subscription<px4_msgs::msg::VehicleLandDetected>(
      get_parameter("px4_land_detected_topic").as_string(), px4_qos,
      [this](const px4_msgs::msg::VehicleLandDetected::SharedPtr msg) {
        if (!msg) return;
        land_detected_ = *msg;
        land_detected_time_ = get_clock()->now();
      });
    takeover_permission_sub_ = create_subscription<std_msgs::msg::Bool>(
      get_parameter("takeover_permission_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Bool::SharedPtr msg) { takeover_permission_ = msg->data; });
    manual_yaw_sub_ = create_subscription<std_msgs::msg::Float64>(
      get_parameter("manual_yaw_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64::SharedPtr msg) {
        if (std::isfinite(msg->data)) manual_yaw_ = msg->data;
      });
    clear_manual_yaw_sub_ = create_subscription<std_msgs::msg::Empty>(
      get_parameter("clear_manual_yaw_topic").as_string(), rclcpp::QoS(10),
      [this](const std_msgs::msg::Empty::SharedPtr) { manual_yaw_.reset(); });

    mode_pub_ = create_publisher<px4_msgs::msg::OffboardControlMode>(
      get_parameter("offboard_control_mode_topic").as_string(), px4_qos);
    setpoint_pub_ = create_publisher<px4_msgs::msg::VehicleAttitudeSetpoint>(
      get_parameter("attitude_setpoint_topic").as_string(), px4_qos);
    vehicle_command_pub_ = create_publisher<px4_msgs::msg::VehicleCommand>(
      get_parameter("vehicle_command_topic").as_string(), px4_qos);
    status_pub_ = create_publisher<std_msgs::msg::String>(
      get_parameter("status_topic").as_string(), rclcpp::QoS(10));
    debug_pub_ = create_publisher<ControlDebug>(
      get_parameter("debug_topic").as_string(), rclcpp::QoS(10));

    const double rate = get_parameter("publish_rate_hz").as_double();
    if (!std::isfinite(rate) || rate < 10.0) throw std::runtime_error("publish_rate_hz must be >= 10");
    timer_ = create_timer(
      std::chrono::duration<double>(1.0 / rate), std::bind(&Px4Ctrl::timer_callback, this));
    publish_status(entry_mode_ == "airborne_takeover" ?
      "waiting_for_airborne_takeover" : "waiting_for_px4_local_position_and_reference_command");
    if (enable_cdm_feedforward_) {
      RCLCPP_INFO(get_logger(),
        "CDM force feed-forward ENABLED (max acceleration %.2f m/s^2; stale-state fallback enabled)",
        cdm_max_acceleration_mps2_);
    } else {
      RCLCPP_INFO(get_logger(), "CDM force feed-forward disabled");
    }
  }

private:
  void command_callback(const DesiredStateMsg::SharedPtr msg) //收参考
  {
    if (!msg || msg->header.frame_id != reference_frame_) return;
    const auto stamp = rclcpp::Time(msg->header.stamp, get_clock()->get_clock_type());
    const double age = (get_clock()->now() - stamp).seconds();
    if (stamp.nanoseconds() <= 0 || !std::isfinite(age) ||
        age > command_timeout_sec_ || age < -0.05 ||
        (last_desired_stamp_ && stamp <= *last_desired_stamp_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Rejecting stale, future, or out-of-order desired state");
      return;
    }
    DesiredState candidate;
    candidate.position = {msg->position.x, msg->position.y, msg->position.z};
    candidate.velocity = {msg->velocity.x, msg->velocity.y, msg->velocity.z};
    candidate.acceleration = {msg->acceleration.x, msg->acceleration.y, msg->acceleration.z};
    candidate.yaw = msg->yaw;
    candidate.yaw_rate = msg->yaw_rate;
    if (!finite(candidate.position) || !finite(candidate.velocity) ||
        !finite(candidate.acceleration) || !std::isfinite(candidate.yaw) ||
        !std::isfinite(candidate.yaw_rate)) return;
    last_desired_stamp_ = stamp;
    const bool has_previous_target = reference_target_.has_value();
    const DesiredState previous_target = reference_target_.value_or(candidate);
    const double position_jump = has_previous_target ?
      (candidate.position - previous_target.position).norm() : 0.0;
    const double velocity_jump = has_previous_target ?
      (candidate.velocity - previous_target.velocity).norm() : 0.0;

    if (has_previous_target &&
      (position_jump > max_command_position_jump_m_ ||
       velocity_jump > max_command_velocity_jump_mps_))
    {
      reference_blend_from_ = desired_.value_or(previous_target);
      reference_blend_start_ = get_clock()->now();
      reference_blending_ = command_blend_sec_ > 1e-6;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Blending discontinuous reference: position jump %.2f m, velocity jump %.2f m/s",
        position_jump, velocity_jump);
    }
    desired_ = candidate;
    reference_target_ = candidate;
    pending_desired_ = candidate;
    desired_command_time_ = get_clock()->now();
    command_seen_ = true;
    if (!first_command_time_) first_command_time_ = desired_command_time_;
  }

  void vehicle_status_callback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
  {
    if (!msg) return;
    vehicle_status_ = *msg;
    vehicle_status_time_ = get_clock()->now();
    update_px4_clock(msg->timestamp);
  }

  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    update_px4_clock(msg->timestamp);
  }

  void update_px4_clock(uint64_t timestamp)
  {
    if (timestamp > px4_timestamp_) {
      px4_timestamp_ = timestamp;
    }
  }

  uint64_t current_px4_timestamp() const
  {
    return px4_timestamp_;
  }

  uint64_t next_setpoint_timestamp()
  {
    uint64_t timestamp = current_px4_timestamp();
    if (timestamp == 0) return 0;
    timestamp = std::max(timestamp, last_setpoint_timestamp_ + 1);
    last_setpoint_timestamp_ = timestamp;
    return timestamp;
  }

  void local_position_callback(const px4_msgs::msg::VehicleLocalPosition::SharedPtr msg) //收反馈+EKF复位补偿
  {
    if (!msg) return;
    const bool local_frame_reset = local_position_ &&
      (msg->xy_reset_counter != local_position_->xy_reset_counter ||
      msg->z_reset_counter != local_position_->z_reset_counter ||
      msg->heading_reset_counter != local_position_->heading_reset_counter);
    if (local_frame_reset) {
      const bool xy_reset = msg->xy_reset_counter != local_position_->xy_reset_counter;
      const bool z_reset = msg->z_reset_counter != local_position_->z_reset_counter;
      const bool heading_reset =
        msg->heading_reset_counter != local_position_->heading_reset_counter;
      const Vector3 reset_delta(
        xy_reset ? static_cast<double>(msg->delta_xy[0]) : 0.0,
        xy_reset ? static_cast<double>(msg->delta_xy[1]) : 0.0,
        z_reset ? static_cast<double>(msg->delta_z) : 0.0);
      const double heading_delta = heading_reset ?
        static_cast<double>(msg->delta_heading) : 0.0;
      if (!finite(reset_delta) || !std::isfinite(heading_delta) ||
        reset_delta.norm() > max_ekf_reset_delta_m_ ||
        std::abs(heading_delta) > max_ekf_reset_heading_rad_)
      {
        latch_safety_fault("px4_local_position_reset_too_large");
      } else {
        hover_position_ += reset_delta;
        takeoff_target_ += reset_delta;
        if (z_reset && takeoff_origin_z_) {
          *takeoff_origin_z_ += reset_delta.z();
        }
        tracking_handoff_position_ += reset_delta;
        heading_reset_offset_ = std::remainder(
          heading_reset_offset_ + heading_delta, 2.0 * M_PI);
        hold_yaw_ = std::remainder(hold_yaw_ + heading_delta, 2.0 * M_PI);
        tracking_handoff_yaw_ = std::remainder(
          tracking_handoff_yaw_ + heading_delta, 2.0 * M_PI);
        if (yaw_setpoint_) {
          *yaw_setpoint_ = std::remainder(*yaw_setpoint_ + heading_delta, 2.0 * M_PI);
        }
        RCLCPP_WARN(get_logger(),
          "Compensating PX4 local reset: delta [%.3f %.3f %.3f] m, heading %.3f rad",
          reset_delta.x(), reset_delta.y(), reset_delta.z(), heading_delta);
      }
    }
    local_position_ = *msg;
    update_px4_clock(msg->timestamp);
    local_position_time_ = get_clock()->now();
    local_position_received_ = true;
  }

  void initialize_cdm_model()
  {
    llc_mpc::ArmCdmGeometry geometry;
    geometry.total_mass = declare_parameter<double>("cdm.total_mass_kg", 2.843243);
    geometry.arm_mass = declare_parameter<double>("cdm.arm_mass_kg", 0.325331);
    geometry.uav_base_inertia = Vector3(
      declare_parameter<double>("cdm.base_inertia_xx", 0.02684785),
      declare_parameter<double>("cdm.base_inertia_yy", 0.02690653),
      declare_parameter<double>("cdm.base_inertia_zz", 0.02385432)).asDiagonal();
    geometry.joint_offset_12 = cdm_vector_parameter(
      "cdm.joint_offset_12_m", {0.198, 0.0005, -0.024});
    geometry.joint_offset_23 = cdm_vector_parameter(
      "cdm.joint_offset_23_m", {0.194, -0.0005, 0.0});
    geometry.link_mass = cdm_vector_parameter(
      "cdm.link_mass_kg", {0.12418, 0.11778, 0.083371});
    const auto centers = declare_parameter<std::vector<double>>(
      "cdm.link_com_m", {0.156980, 0.019388, -0.0082101,
        0.144410, 0.018876, 8.2381e-7,
        0.071417896813, 0.018996302624, 0.010302221502});
    const auto inertias = declare_parameter<std::vector<double>>(
      "cdm.link_inertia_kg_m2", {
        3.0533e-5, 1.8411e-9, -1.1909e-9, 1.8411e-9, 8.0863e-5, 1.5749e-8,
        -1.1909e-9, 1.5749e-8, 7.8412e-5,
        1.9771e-5, 3.3851e-8, -5.9263e-11, 3.3851e-8, 5.6465e-5, 1.1813e-13,
        -5.9263e-11, 1.1813e-13, 6.3243e-5,
        3.063625451122e-5, 8.370556733279e-9, 2.758764487371e-6,
        8.370556733279e-9, 7.715607141505e-5, -2.583337077642e-9,
        2.758764487371e-6, -2.583337077642e-9, 5.766176007008e-5});
    if (centers.size() != 9 || inertias.size() != 27) {
      throw std::runtime_error("CDM link COM/inertia parameter lengths must be 9 and 27");
    }
    for (int link = 0; link < 3; ++link) {
      geometry.local_com[link] = Vector3(
        centers[3 * link], centers[3 * link + 1], centers[3 * link + 2]);
      for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
          geometry.local_com_inertia[link](row, col) =
            inertias[9 * link + 3 * row + col];
        }
      }
    }
    cdm_model_ = std::make_unique<llc_mpc::ArmCdmParameters>(geometry);
    if (!cdm_model_->valid()) {
      throw std::runtime_error("Invalid CDM geometry parameters");
    }
    cdm_total_mass_ = geometry.total_mass;
  }

  Vector3 cdm_vector_parameter(const std::string & name, const std::array<double, 3> & fallback)
  {
    const auto values = declare_parameter<std::vector<double>>(
      name, {fallback[0], fallback[1], fallback[2]});
    if (values.size() != 3) throw std::runtime_error(name + " must have 3 elements");
    return Vector3(values[0], values[1], values[2]);
  }

  static Vector3 cdm_frd_to_flu(const Vector3 & value)
  {
    return Vector3(value.x(), -value.y(), -value.z());
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    const std::array<std::string, 3> names{{"shoulder_pitch", "elbow_pitch", "wrist_pitch"}};
    Vector3 q, dq;
    for (size_t joint = 0; joint < names.size(); ++joint) {
      const auto it = std::find(msg->name.begin(), msg->name.end(), names[joint]);
      if (it == msg->name.end()) return;
      const auto index = static_cast<size_t>(it - msg->name.begin());
      if (index >= msg->position.size() || index >= msg->velocity.size()) return;
      q(joint) = msg->position[index];
      dq(joint) = msg->velocity[index];
    }
    if (!q.allFinite() || !dq.allFinite()) return;
    const auto now = std::chrono::steady_clock::now();
    if (cdm_arm_state_valid_) {
      const double dt = std::chrono::duration<double>(now - cdm_joint_update_).count();
      if (dt > 0.002 && dt < 0.2) {
        const Vector3 raw = (dq - cdm_arm_dq_) / dt;
        cdm_arm_ddq_ = 0.7 * cdm_arm_ddq_ + 0.3 * raw;
      }
    }
    cdm_arm_q_ = q;
    cdm_arm_dq_ = dq;
    cdm_joint_update_ = now;
    cdm_arm_state_valid_ = true;
  }

  void cdm_odometry_callback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg)
  {
    if (msg->pose_frame != px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED) return;
    Eigen::Quaterniond q(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    if (!q.coeffs().allFinite() || q.norm() < 0.5) return;
    q.normalize();
    cdm_frd_to_ned_ = q;
    Vector3 velocity(msg->velocity[0], msg->velocity[1], msg->velocity[2]);
    if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED) {
      cdm_velocity_flu_ = cdm_frd_to_flu(q.conjugate() * velocity);
    } else if (msg->velocity_frame == px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD) {
      cdm_velocity_flu_ = cdm_frd_to_flu(velocity);
    } else {
      cdm_velocity_flu_.setZero();
    }
    const Vector3 omega = cdm_frd_to_flu(Vector3(
      msg->angular_velocity[0], msg->angular_velocity[1], msg->angular_velocity[2]));
    const auto now = std::chrono::steady_clock::now();
    if (cdm_odometry_valid_) {
      const double dt = std::chrono::duration<double>(now - cdm_odom_update_).count();
      if (dt > 0.002 && dt < 0.2) {
        cdm_alpha_flu_ = 0.8 * cdm_alpha_flu_ + 0.2 * (omega - cdm_omega_flu_) / dt;
      }
    }
    cdm_omega_flu_ = omega;
    cdm_odom_update_ = now;
    cdm_odometry_valid_ = cdm_velocity_flu_.allFinite() && omega.allFinite();
  }

  Vector3 cdm_acceleration_feedforward()
  {
    if (!enable_cdm_feedforward_) return Vector3::Zero();
    const auto now = std::chrono::steady_clock::now();
    if (!cdm_model_ || !cdm_arm_state_valid_ || !cdm_odometry_valid_ ||
      now - cdm_joint_update_ > std::chrono::duration<double>(cdm_state_timeout_sec_) ||
      now - cdm_odom_update_ > std::chrono::duration<double>(cdm_state_timeout_sec_))
    {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "CDM feed-forward enabled but arm/odometry is stale; using feedback only");
      return Vector3::Zero();
    }
    llc_mpc::ArmJointPrediction joints;
    joints.q = cdm_arm_q_;
    joints.dq = cdm_arm_dq_;
    joints.ddq = cdm_arm_ddq_;
    llc_mpc::ArmStage stage;
    llc_mpc::CdmDisturbance disturbance;
    if (!cdm_model_->calculate(joints, stage) ||
      !cdm_model_->disturbance(stage, cdm_omega_flu_, cdm_alpha_flu_,
        cdm_velocity_flu_, cdm_total_mass_ * gravity_, disturbance))
    {
      return Vector3::Zero();
    }
    const Vector3 force_frd = cdm_frd_to_flu(disturbance.force_body);
    Vector3 accel_ned = -(cdm_frd_to_ned_ * force_frd) / cdm_total_mass_;
    if (!accel_ned.allFinite()) return Vector3::Zero();
    const double norm = accel_ned.norm();
    if (norm > cdm_max_acceleration_mps2_) accel_ned *= cdm_max_acceleration_mps2_ / norm;
    return accel_ned;
  }

  bool local_position_healthy() const //反馈是否可信
  {
    if (!require_local_position_valid_) return true;
    if (!local_position_received_ || !local_position_time_) return false;
    if (seconds_since(*local_position_time_) > local_position_timeout_sec_) return false;
    const auto & p = *local_position_;
    if (p.timestamp == 0 || !p.xy_valid || !p.z_valid || !p.v_xy_valid || !p.v_z_valid || p.dead_reckoning) return false;
    const double position_norm = std::hypot(std::hypot(static_cast<double>(p.x), static_cast<double>(p.y)),
      static_cast<double>(p.z));
    const double velocity_norm = std::hypot(std::hypot(static_cast<double>(p.vx), static_cast<double>(p.vy)),
      static_cast<double>(p.vz));
    return std::isfinite(position_norm) && std::isfinite(velocity_norm) &&
      position_norm <= max_local_position_m_ && velocity_norm <= max_local_velocity_mps_;
  }

  bool vehicle_status_fresh() const
  {
    return vehicle_status_ && vehicle_status_time_ && vehicle_status_->timestamp != 0 &&
      seconds_since(*vehicle_status_time_) <= px4_status_timeout_sec_;
  }

  bool vehicle_armed() const
  {
    return vehicle_status_fresh() &&
      vehicle_status_->arming_state == px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED;
  }

  bool vehicle_offboard() const
  {
    return vehicle_status_fresh() &&
      vehicle_status_->nav_state == px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD;
  }

  double seconds_since(const rclcpp::Time & earlier) const
  {
    return std::max(0.0, (get_clock()->now() - earlier).seconds());
  }

  void latch_safety_fault(const std::string & reason)
  {
    if (safety_latched_) return;
    safety_latched_ = true;
    state_ = ControlState::safety_fault;
    publish_status("safety_fault_" + reason);
    RCLCPP_ERROR(get_logger(), "PX4 control latched safety fault: %s", reason.c_str());
    if (land_on_safety_fault_ && vehicle_status_ &&
      vehicle_status_->arming_state != px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_NAV_LAND, 0.0F, 0.0F);
    }
  }

  bool state_fresh() const
  {
    // PX4 local position is the only control-state source. The reference is
    // already expressed in the same frame, so no second odometry stream is
    // needed to establish or maintain a common origin.
    return local_position_received_ && local_position_time_ &&
      seconds_since(*local_position_time_) <= local_position_timeout_sec_;
  }

  bool command_fresh() const
  {
    return pending_desired_ && desired_command_time_ &&
      seconds_since(*desired_command_time_) <= command_timeout_sec_;
  }

  ControlOutput calculate_output()
  {
    const auto & lpos = *local_position_;
    const Vector3 p(static_cast<double>(lpos.x), static_cast<double>(lpos.y), static_cast<double>(lpos.z));
    const Vector3 v(static_cast<double>(lpos.vx), static_cast<double>(lpos.vy), static_cast<double>(lpos.vz));
    DesiredState d = desired_.value_or(DesiredState{});

    // Blend a discontinuous replan before it reaches the attitude loop. A
    // planner that replaces its reference with a state-continuous command
    // never triggers this; only a discontinuity such as a planner reset does.
    if (reference_blending_ && reference_blend_start_ && command_blend_sec_ > 1e-6) {
      const double elapsed = seconds_since(*reference_blend_start_);
      const double u = std::clamp(elapsed / command_blend_sec_, 0.0, 1.0);
      const double blend = u * u * (3.0 - 2.0 * u);
      const DesiredState & from = reference_blend_from_;
      d.position = (1.0 - blend) * from.position + blend * d.position;
      d.velocity = (1.0 - blend) * from.velocity + blend * d.velocity;
      d.acceleration = (1.0 - blend) * from.acceleration + blend * d.acceleration;
      d.yaw = from.yaw + blend * std::remainder(d.yaw - from.yaw, 2.0 * M_PI);
      d.yaw_rate = (1.0 - blend) * from.yaw_rate + blend * d.yaw_rate;
      if (u >= 1.0) reference_blending_ = false;
    }

    if (entry_mode_ != "airborne_takeover" && state_ == ControlState::tracking &&
        takeoff_origin_z_) {
      // The reference can briefly publish a failed/recovery trajectory below
      // the safe flight floor. Keep the direct attitude controller from
      // commanding a descent while the planner waits for a valid replacement.
      const double minimum_z = *takeoff_origin_z_ - minimum_tracking_altitude_m_;
      if (d.position.z() > minimum_z) {
        // In NED, smaller z means higher altitude. If already above the
        // floor, hold the current altitude instead of descending to it.
        d.position.z() = std::min(p.z(), minimum_z);
        if (d.velocity.z() > 0.0) d.velocity.z() = 0.0;
        if (d.acceleration.z() > 0.0) d.acceleration.z() = 0.0;
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
          "Clamping reference to minimum tracking altitude %.2f m", minimum_tracking_altitude_m_);
      }
    }
    // The reference already uses PX4 local NED, so yaw and yaw rate pass
    // through unchanged. Only the estimator heading-reset offset is applied.
    d.yaw = std::remainder(d.yaw + heading_reset_offset_, 2.0 * M_PI);
    if (!command_fresh()) {
      d.velocity.setZero();
      d.acceleration.setZero();
      d.yaw_rate = 0.0;
    } else if (state_ == ControlState::tracking && tracking_handoff_start_time_ &&
      tracking_handoff_sec_ > 1e-6)
    {
      const double elapsed = seconds_since(*tracking_handoff_start_time_);
      const double u = std::clamp(elapsed / tracking_handoff_sec_, 0.0, 1.0);
      const double blend = u * u * (3.0 - 2.0 * u);
      d.position = (1.0 - blend) * tracking_handoff_position_ + blend * d.position;
      d.velocity = (1.0 - blend) * tracking_handoff_velocity_ + blend * d.velocity;
      d.acceleration *= blend;
      d.yaw = tracking_handoff_yaw_ + blend * std::remainder(
        d.yaw - tracking_handoff_yaw_, 2.0 * M_PI);
      d.yaw_rate *= blend;
    }
    // Keep the launch heading while takeoff and hover settling establish lift.
    // A reference may request a large turn, which should start only once the
    // vehicle is airborne and the reference is being tracked.
    double yaw_target = state_ == ControlState::tracking ? d.yaw : hold_yaw_;
    if (state_ == ControlState::tracking && manual_yaw_) {
      yaw_target = std::remainder(*manual_yaw_ + heading_reset_offset_, 2.0 * M_PI);
    }
    const auto yaw_now = get_clock()->now();
    const double yaw_dt = last_yaw_update_ ? std::clamp(
      (yaw_now - *last_yaw_update_).seconds(), 0.0, 0.05) : 0.0;
    if (!yaw_setpoint_) {
      yaw_setpoint_ = std::isfinite(lpos.heading) ? static_cast<double>(lpos.heading) : yaw_target;
    }
    double yaw_step = 0.0;
    if (yaw_dt > 1e-6) {
      const double yaw_error = std::remainder(yaw_target - *yaw_setpoint_, 2.0 * M_PI);
      const double yaw_feedforward = (state_ == ControlState::tracking && !manual_yaw_) ?
        d.yaw_rate : 0.0;
      const double desired_yaw_rate = std::clamp(
        yaw_feedforward + 2.0 * yaw_error, -max_yaw_rate_radps_, max_yaw_rate_radps_);
      const double max_yaw_rate_step = max_yaw_acceleration_radps2_ * yaw_dt;
      yaw_rate_setpoint_ += std::clamp(
        desired_yaw_rate - yaw_rate_setpoint_, -max_yaw_rate_step, max_yaw_rate_step);
      yaw_rate_setpoint_ = std::clamp(
        yaw_rate_setpoint_, -max_yaw_rate_radps_, max_yaw_rate_radps_);
      yaw_step = yaw_rate_setpoint_ * yaw_dt;
    }
    *yaw_setpoint_ = std::remainder(*yaw_setpoint_ + yaw_step, 2.0 * M_PI);
    last_yaw_update_ = yaw_now;
    const Vector3 position_error = d.position - p;
    // kp is the historical acceleration-per-position gain. Divide by kv so
    // existing YAML preserves its proportional acceleration contribution.
    Vector3 velocity_reference = d.velocity;
    for (int i = 0; i < 3; ++i) {
      if (kv_(i) > 1e-9) velocity_reference(i) += kp_(i) / kv_(i) * position_error(i);
    }
    const Vector3 velocity_error = velocity_reference - v;
    const bool integral_enabled = state_ == ControlState::tracking && command_fresh() &&
      velocity_error.norm() <= integral_enable_velocity_error_mps_;
    const Vector3 integral_before = integral_acceleration_;
    if (!integral_enabled || last_output_saturated_) {
      integral_acceleration_ *= 0.995;
    } else {
      const double dt = last_control_time_ ? std::clamp(
        (get_clock()->now() - *last_control_time_).seconds(), 0.0, 0.05) : 0.0;
      integral_acceleration_ += ki_.cwiseProduct(velocity_error) * dt;
      for (int i = 0; i < 3; ++i) {
        integral_acceleration_(i) = std::clamp(
          integral_acceleration_(i), -integral_limit_(i), integral_limit_(i));
      }
    }
    const Vector3 integral_term = integral_acceleration_;
    Vector3 acceleration = d.acceleration + cdm_acceleration_feedforward() +
      kv_.cwiseProduct(velocity_error) + integral_term;
    for (int i = 0; i < 3; ++i) {
      if (kv_(i) <= 1e-9) acceleration(i) += kp_(i) * position_error(i);
    }
    if (!acceleration.allFinite()) acceleration.setZero();
    const Vector3 acceleration_requested = acceleration;
    const double acceleration_norm = acceleration.norm();
    uint8_t saturation_flags = 0;
    if (acceleration_norm > max_acceleration_) {
      acceleration *= max_acceleration_ / acceleration_norm;
      saturation_flags |= 1;
    }

    // Project the requested acceleration into the physical thrust/tilt
    // envelope. Hard-clipping collective thrust after attitude generation
    // keeps an unreachable acceleration target in the loop and causes large
    // overshoot when hover thrust is close to the configured maximum.
    const double max_force = gravity_ * max_thrust_ / hover_thrust_;
    const double min_force = gravity_ * min_thrust_ / hover_thrust_;
    const double requested_vertical_force = gravity_ - acceleration.z();
    const double vertical_force = std::clamp(requested_vertical_force, min_force, max_force);
    if (std::abs(vertical_force - requested_vertical_force) > 1e-9) saturation_flags |= 2;
    acceleration.z() = gravity_ - vertical_force;

    Eigen::Vector2d horizontal_acceleration = acceleration.head<2>();
    const double horizontal_norm = horizontal_acceleration.norm();
    const double horizontal_force_from_tilt = std::tan(max_tilt_rad_) * vertical_force;
    const double horizontal_force_from_thrust = std::sqrt(std::max(
      0.0, max_force * max_force - vertical_force * vertical_force));
    const double max_horizontal_acceleration = std::min(
      horizontal_force_from_tilt, horizontal_force_from_thrust);
    if (horizontal_norm > max_horizontal_acceleration && horizontal_norm > 1e-9) {
      horizontal_acceleration *= max_horizontal_acceleration / horizontal_norm;
      acceleration.head<2>() = horizontal_acceleration;
      saturation_flags |= 4;
    }

    // NED specific force required from the vehicle. PX4's position controller
    // uses the same expression: force = acceleration - [0, 0, g].
    Vector3 force = acceleration - Vector3(0.0, 0.0, gravity_);
    if (!force.allFinite() || force.norm() < 1e-9) {
      force = Vector3(0.0, 0.0, -gravity_);
    }
    // Match PX4's limitTilt() before converting the vertical component into a
    // collective thrust. This preserves the requested vertical acceleration
    // when horizontal demand reaches the tilt limit.
    Vector3 body_z = limit_body_z_tilt(-force, max_tilt_rad_);
    const double force_down = std::max(0.1, gravity_ - acceleration.z());
    const double body_z_down = std::max(1e-3, body_z.z());
    double thrust = hover_thrust_ * force_down / (gravity_ * body_z_down);
    const double unclamped_thrust = thrust;
    thrust = std::clamp(thrust, min_thrust_, max_thrust_);
    if (std::abs(unclamped_thrust - thrust) > 1e-9) saturation_flags |= 8;
    if (saturation_flags != 0) {
      integral_acceleration_ = integral_before * 0.995;
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "Acceleration projected into thrust envelope; desired acceleration [%.2f %.2f %.2f] m/s^2",
        acceleration.x(), acceleration.y(), acceleration.z());
    }

    ControlOutput out;
    out.attitude = body_z_and_yaw_to_attitude(body_z, *yaw_setpoint_);
    out.thrust = thrust;
    out.tilt_deg = std::acos(std::clamp(body_z.z(), -1.0, 1.0)) * 180.0 / M_PI;
    out.saturation_flags = saturation_flags;
    out.yaw_rate = yaw_rate_setpoint_;
    out.position_error = position_error;
    out.velocity_error = velocity_error;
    out.velocity_reference = velocity_reference;
    out.acceleration_requested = acceleration_requested;
    out.acceleration_feasible = acceleration;
    last_output_saturated_ = saturation_flags != 0;
    last_control_time_ = get_clock()->now();
    return out;
  }

  void begin_takeoff()
  {
    if (!local_position_) return;
    const auto & p = *local_position_;
    hover_position_ = {static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
    hold_yaw_ = std::isfinite(p.heading) ? static_cast<double>(p.heading) : 0.0;
    takeoff_target_ = hover_position_;
    takeoff_origin_z_ = hover_position_.z();
    // PX4 local z is Down, therefore a positive altitude is a negative z target.
    takeoff_target_.z() -= auto_takeoff_altitude_m_;
    takeoff_start_time_ = get_clock()->now();
    hover_settle_start_time_.reset();
    state_ = auto_takeoff_altitude_m_ > 1e-3 ? ControlState::takeoff : ControlState::hover;
    publish_status(state_ == ControlState::takeoff ? "state_takeoff" : "state_hover");
  }

  bool begin_airborne_takeover()
  {
    if (!local_position_healthy() || !vehicle_armed() || vehicle_status_->failsafe) {
      takeover_stable_since_.reset();
      return false;
    }
    if (!land_detected_ || !land_detected_time_ || land_detected_->timestamp == 0 ||
        seconds_since(*land_detected_time_) > land_detected_timeout_sec_ ||
        land_detected_->landed || land_detected_->ground_contact ||
        land_detected_->maybe_landed) {
      takeover_stable_since_.reset();
      return false;
    }
    const auto & p = *local_position_;
    const Vector3 velocity{static_cast<double>(p.vx), static_cast<double>(p.vy),
      static_cast<double>(p.vz)};
    if (velocity.norm() > takeover_velocity_tolerance_mps_ || !std::isfinite(p.heading)) {
      takeover_stable_since_.reset();
      return false;
    }
    if (!takeover_stable_since_) {
      takeover_stable_since_ = get_clock()->now();
      return false;
    }
    if (seconds_since(*takeover_stable_since_) < std::max(1.0, hover_settle_sec_)) return false;
    if (require_takeover_permission_ && !takeover_permission_) return false;
    hover_position_ = {static_cast<double>(p.x), static_cast<double>(p.y),
      static_cast<double>(p.z)};
    hold_yaw_ = static_cast<double>(p.heading);
    yaw_setpoint_ = hold_yaw_;
    yaw_rate_setpoint_ = 0.0;
    takeoff_origin_z_.reset();
    hover_settle_start_time_ = get_clock()->now();
    takeover_start_time_ = get_clock()->now();
    state_ = ControlState::hover;
    publish_status("hover_airborne_takeover_prestream");
    RCLCPP_INFO(get_logger(), "Captured airborne hold in PX4 NED [%.2f %.2f %.2f] m",
      hover_position_.x(), hover_position_.y(), hover_position_.z());
    return true;
  }

  void update_state_machine()
  {
    if (!local_position_) return;
    const auto & p = *local_position_;
    const Vector3 current{static_cast<double>(p.x), static_cast<double>(p.y), static_cast<double>(p.z)};
    const Vector3 velocity{static_cast<double>(p.vx), static_cast<double>(p.vy), static_cast<double>(p.vz)};
    if (state_ == ControlState::waiting_for_command) {
      if (entry_mode_ == "airborne_takeover") {
        begin_airborne_takeover();
      } else if (command_seen_) {
        begin_takeoff();
      }
    }

    if (state_ == ControlState::takeoff) {
      const double climb_m = hover_position_.z() - current.z();
      // The payload's calibrated hover point can be below the nominal target.
      // Release the takeoff gate once it has either reached the requested
      // height or demonstrated a safe minimum climb above the launch plane.
      const bool at_height = (current - takeoff_target_).norm() <= takeoff_tolerance_m_;
      const bool safely_airborne = climb_m >= minimum_takeoff_altitude_m_;
      const bool settled = velocity.norm() <= hover_velocity_tolerance_mps_;
      const double elapsed = seconds_since(*takeoff_start_time_);
      if ((at_height || safely_airborne) && settled && elapsed >= takeoff_duration_sec_) {
        state_ = ControlState::hover;
        hover_position_ = current;
        hover_settle_start_time_ = get_clock()->now();
        publish_status(safely_airborne && !at_height ? "state_hover_minimum_altitude" : "state_hover");
      }
    }
    if (state_ == ControlState::hover) {
      if (!hover_settle_start_time_) hover_settle_start_time_ = get_clock()->now();
      if (velocity.norm() > hover_velocity_tolerance_mps_) {
        hover_settle_start_time_ = get_clock()->now();
      }
      const double settled_for = seconds_since(*hover_settle_start_time_);
      if (command_fresh() && pending_desired_ &&
          (entry_mode_ == "airborne_takeover" || settled_for >= hover_settle_sec_) &&
          (entry_mode_ != "airborne_takeover" || vehicle_offboard())) {
        state_ = ControlState::tracking;
        desired_ = pending_desired_;
        tracking_handoff_position_ = current;
        tracking_handoff_velocity_ = velocity;
        tracking_handoff_yaw_ = std::isfinite(p.heading) ? static_cast<double>(p.heading) : hold_yaw_;
        tracking_handoff_start_time_ = get_clock()->now();
        publish_status("state_tracking");
      }
    }
    if (state_ == ControlState::tracking && !command_fresh()) {
      hover_position_ = current;
      hover_settle_start_time_ = get_clock()->now();
      state_ = ControlState::hover;
      publish_status("state_hover_command_timeout");
    }
  }

  void timer_callback()
  {
    if (safety_latched_) {
      publish_status("safety_latched_no_setpoint");
      return;
    }
    if (control_suspended_) {
      publish_status("control_suspended_" + suspension_reason_);
      return;
    }
    if (!vehicle_status_fresh()) {
      if (state_ != ControlState::waiting_for_command) {
        control_suspended_ = true;
        suspension_reason_ = "px4_status_timeout";
      }
      publish_status("waiting_for_fresh_px4_status");
      return;
    }
    if (vehicle_status_->failsafe) {
      control_suspended_ = true;
      suspension_reason_ = "px4_failsafe";
      publish_status("control_suspended_px4_failsafe");
      return;
    }
    if (offboard_seen_ && !vehicle_offboard()) {
      control_suspended_ = true;
      suspension_reason_ = "offboard_lost";
      publish_status("control_suspended_offboard_lost");
      return;
    }
    if (entry_mode_ == "airborne_takeover" && state_ != ControlState::waiting_for_command &&
        !vehicle_armed())
    {
      control_suspended_ = true;
      suspension_reason_ = "vehicle_disarmed";
      publish_status("control_suspended_vehicle_disarmed");
      return;
    }
    if (vehicle_offboard() && state_ != ControlState::waiting_for_command) offboard_seen_ = true;
    const auto now = get_clock()->now();
    if (!state_fresh()) {
      if (vehicle_status_ && vehicle_status_->arming_state !=
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED)
      {
        if (!unhealthy_since_) unhealthy_since_ = now;
        if (std::max(0.0, (now - *unhealthy_since_).seconds()) >=
          local_position_fault_delay_sec_)
        {
          latch_safety_fault("px4_local_position_timeout");
        }
      } else {
        publish_status("waiting_for_fresh_px4_local_position");
      }
      return;
    }
    if (!local_position_healthy()) {
      publish_status("waiting_for_healthy_px4_local_position");
      if (vehicle_status_ && vehicle_status_->arming_state !=
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED) {
        if (!unhealthy_since_) unhealthy_since_ = now;
        if (std::max(0.0, (now - *unhealthy_since_).seconds()) >=
          local_position_fault_delay_sec_)
        {
          latch_safety_fault("px4_local_position_unhealthy");
        }
      }
      return;
    }
    unhealthy_since_.reset();

    update_state_machine();
    if (state_ == ControlState::waiting_for_command) {
      publish_status(entry_mode_ == "airborne_takeover" ?
        "waiting_for_armed_stationary_airborne_state" : "waiting_for_command_waiting_for_goal");
      return;
    }
    px4_msgs::msg::OffboardControlMode mode;
    mode.timestamp = current_px4_timestamp();
    mode.attitude = true;
    mode_pub_->publish(mode);
    if (state_ == ControlState::hover || state_ == ControlState::takeoff) {
      DesiredState hold;
      const Vector3 target = state_ == ControlState::takeoff ? takeoff_target_ : hover_position_;
      // The hold target is already a PX4 local NED position.
      hold.position = target;
      hold.velocity.setZero();
      hold.acceleration.setZero();
      hold.yaw = manual_yaw_.value_or(hold_yaw_);
      desired_ = hold;
    }
    if (state_ == ControlState::tracking && pending_desired_) {
      desired_ = pending_desired_;
    }
    const ControlOutput output = calculate_output();
    px4_msgs::msg::VehicleAttitudeSetpoint setpoint;
    const uint64_t setpoint_timestamp = next_setpoint_timestamp();
    if (setpoint_timestamp == 0) {
      publish_status("waiting_for_px4_clock");
      return;
    }
    setpoint.timestamp = setpoint_timestamp;
    setpoint.q_d = {static_cast<float>(output.attitude.w()), static_cast<float>(output.attitude.x()),
                    static_cast<float>(output.attitude.y()), static_cast<float>(output.attitude.z())};
    setpoint.thrust_body = {0.0F, 0.0F, static_cast<float>(-output.thrust)};
    setpoint.yaw_sp_move_rate = static_cast<float>(output.yaw_rate);
    setpoint_pub_->publish(setpoint);
    publish_debug(output);
    if (entry_mode_ == "airborne_takeover") {
      if (!vehicle_offboard()) {
        const bool prestream_ready = takeover_start_time_ &&
          seconds_since(*takeover_start_time_) >= prestream_seconds_;
        publish_status(prestream_ready ? "hover_airborne_takeover_ready" :
          "hover_airborne_takeover_prestream");
      } else if (state_ == ControlState::hover) {
        const bool prestream_ready = takeover_start_time_ &&
          seconds_since(*takeover_start_time_) >= prestream_seconds_;
        publish_status(prestream_ready ? "hover_airborne_takeover_offboard_ready" :
          "hover_airborne_takeover_prestream");
      } else {
        publish_status("tracking_attitude_thrust_tracking");
      }
      return;
    }
    if (!command_seen_ || !start_offboard_ || !vehicle_status_) return;
    const double elapsed = seconds_since(*first_command_time_);
    if (elapsed < prestream_seconds_) return;
    if (vehicle_status_->nav_state != px4_msgs::msg::VehicleStatus::NAVIGATION_STATE_OFFBOARD) {
      if (retry_due()) {
        publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1.0F, 6.0F);
        last_command_time_ = get_clock()->now();
        publish_status("attitude_offboard_requested");
      }
      return;
    }
    if (arm_on_first_ && vehicle_status_->arming_state ==
        px4_msgs::msg::VehicleStatus::ARMING_STATE_DISARMED &&
        vehicle_status_->pre_flight_checks_pass && retry_due()) {
      publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0F, 0.0F);
      last_command_time_ = get_clock()->now();
      publish_status("attitude_offboard_arm_requested");
    }
    const std::string phase = control_state_name();
    publish_status(command_fresh() ? phase + "_attitude_thrust_tracking" :
      phase + "_command_stale_hovering");
  }

  std::string control_state_name() const
  {
    switch (state_) {
      case ControlState::waiting_for_command: return "waiting_for_command";
      case ControlState::takeoff: return "takeoff";
      case ControlState::hover: return "hover";
      case ControlState::tracking: return "tracking";
      case ControlState::safety_fault: return "safety_fault";
    }
    return "unknown";
  }

  bool retry_due() const
  {
    return !last_command_time_ || seconds_since(*last_command_time_) >= 1.0;
  }

  void publish_vehicle_command(uint16_t command, float p1, float p2)
  {
    px4_msgs::msg::VehicleCommand msg;
    msg.timestamp = current_px4_timestamp();
    msg.param1 = p1;
    msg.param2 = p2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    vehicle_command_pub_->publish(msg);
  }

  void publish_status(const std::string & text)
  {
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  void publish_debug(const ControlOutput & output)
  {
    auto to_msg = [](const Vector3 & value) {
        geometry_msgs::msg::Vector3 msg;
        msg.x = value.x();
        msg.y = value.y();
        msg.z = value.z();
        return msg;
      };
    ControlDebug msg;
    msg.header.stamp = get_clock()->now();
    msg.header.frame_id = "px4_local_ned";
    msg.state = control_state_name();
    msg.position_error = to_msg(output.position_error);
    msg.velocity_error = to_msg(output.velocity_error);
    msg.velocity_reference = to_msg(output.velocity_reference);
    msg.acceleration_requested = to_msg(output.acceleration_requested);
    msg.acceleration_feasible = to_msg(output.acceleration_feasible);
    msg.thrust = static_cast<float>(output.thrust);
    msg.tilt_deg = static_cast<float>(output.tilt_deg);
    msg.yaw_rate = static_cast<float>(output.yaw_rate);
    msg.command_age_sec = desired_command_time_ ?
      static_cast<float>(seconds_since(*desired_command_time_)) : -1.0F;
    msg.saturation_flags = output.saturation_flags;
    msg.local_valid = local_position_healthy();
    msg.status_fresh = vehicle_status_fresh();
    msg.armed = vehicle_armed();
    msg.offboard = vehicle_offboard();
    msg.failsafe = vehicle_status_ && vehicle_status_->failsafe;
    debug_pub_->publish(msg);
  }

  std::string reference_frame_;
  std::string entry_mode_{"auto_takeoff"};
  std::string suspension_reason_;
  bool enable_cdm_feedforward_{false};
  double cdm_state_timeout_sec_{0.20};
  double cdm_max_acceleration_mps2_{1.0};
  double cdm_total_mass_{2.843243};
  std::unique_ptr<llc_mpc::ArmCdmParameters> cdm_model_;
  Vector3 cdm_arm_q_{Vector3::Zero()};
  Vector3 cdm_arm_dq_{Vector3::Zero()};
  Vector3 cdm_arm_ddq_{Vector3::Zero()};
  Vector3 cdm_velocity_flu_{Vector3::Zero()};
  Vector3 cdm_omega_flu_{Vector3::Zero()};
  Vector3 cdm_alpha_flu_{Vector3::Zero()};
  Eigen::Quaterniond cdm_frd_to_ned_{Eigen::Quaterniond::Identity()};
  bool cdm_arm_state_valid_{false};
  bool cdm_odometry_valid_{false};
  std::chrono::steady_clock::time_point cdm_joint_update_{};
  std::chrono::steady_clock::time_point cdm_odom_update_{};
  double command_timeout_sec_{0.5};
  double prestream_seconds_{1.0};
  double takeover_velocity_tolerance_mps_{0.20};
  bool require_takeover_permission_{false};
  bool takeover_permission_{false};
  bool start_offboard_{true};
  bool arm_on_first_{true};
  double gravity_{9.81};
  double hover_thrust_{0.864};
  double min_thrust_{0.08};
  double max_thrust_{0.95};
  double max_tilt_rad_{0.52};
  double max_acceleration_{4.0};
  double local_position_timeout_sec_{0.5};
  double land_detected_timeout_sec_{2.0};
  double px4_status_timeout_sec_{1.5};
  double local_position_fault_delay_sec_{1.0};
  double max_ekf_reset_delta_m_{1.0};
  double max_ekf_reset_heading_rad_{0.35};
  bool require_local_position_valid_{true};
  double max_local_position_m_{100.0};
  double max_local_velocity_mps_{15.0};
  bool land_on_safety_fault_{true};
  double auto_takeoff_altitude_m_{0.5};
  double minimum_takeoff_altitude_m_{0.25};
  double minimum_tracking_altitude_m_{0.25};
  double takeoff_duration_sec_{3.0};
  double takeoff_tolerance_m_{0.08};
  double hover_settle_sec_{0.8};
  double hover_velocity_tolerance_mps_{0.15};
  double tracking_handoff_sec_{0.75};
  double command_blend_sec_{0.25};
  double max_command_position_jump_m_{1.0};
  double max_command_velocity_jump_mps_{1.5};
  double max_yaw_rate_radps_{0.6};
  double max_yaw_acceleration_radps2_{0.8};
  Vector3 kp_{1.5, 1.5, 1.8};
  Vector3 kv_{1.2, 1.2, 1.4};
  Vector3 ki_{0.10, 0.10, 0.15};
  Vector3 integral_limit_{0.40, 0.40, 0.50};
  Vector3 integral_acceleration_{Vector3::Zero()};
  double integral_enable_velocity_error_mps_{0.60};
  bool last_output_saturated_{false};
  std::optional<rclcpp::Time> last_control_time_;
  std::optional<DesiredState> desired_;
  std::optional<DesiredState> reference_target_;
  DesiredState reference_blend_from_;
  std::optional<rclcpp::Time> reference_blend_start_;
  bool reference_blending_{false};
  std::optional<DesiredState> pending_desired_;
  std::optional<rclcpp::Time> desired_command_time_;
  std::optional<rclcpp::Time> first_command_time_;
  std::optional<rclcpp::Time> last_desired_stamp_;
  std::optional<rclcpp::Time> last_command_time_;
  std::optional<double> manual_yaw_;
  std::optional<double> yaw_setpoint_;
  double yaw_rate_setpoint_{0.0};
  std::optional<rclcpp::Time> last_yaw_update_;
  std::optional<px4_msgs::msg::VehicleStatus> vehicle_status_;
  std::optional<rclcpp::Time> vehicle_status_time_;
  std::optional<px4_msgs::msg::VehicleLocalPosition> local_position_;
  std::optional<rclcpp::Time> local_position_time_;
  std::optional<px4_msgs::msg::VehicleLandDetected> land_detected_;
  std::optional<rclcpp::Time> land_detected_time_;
  std::optional<rclcpp::Time> unhealthy_since_;
  bool local_position_received_{false};
  bool safety_latched_{false};
  bool control_suspended_{false};
  bool offboard_seen_{false};
  bool command_seen_{false};
  ControlState state_{ControlState::waiting_for_command};
  Vector3 hover_position_{Vector3::Zero()};
  Vector3 takeoff_target_{Vector3::Zero()};
  std::optional<double> takeoff_origin_z_;
  double hold_yaw_{0.0};
  double heading_reset_offset_{0.0};
  std::optional<rclcpp::Time> takeoff_start_time_;
  std::optional<rclcpp::Time> takeover_start_time_;
  std::optional<rclcpp::Time> takeover_stable_since_;
  std::optional<rclcpp::Time> hover_settle_start_time_;
  Vector3 tracking_handoff_position_{Vector3::Zero()};
  Vector3 tracking_handoff_velocity_{Vector3::Zero()};
  double tracking_handoff_yaw_{0.0};
  std::optional<rclcpp::Time> tracking_handoff_start_time_;
  uint64_t px4_timestamp_{0};
  uint64_t last_setpoint_timestamp_{0};
  rclcpp::Subscription<DesiredStateMsg>::SharedPtr command_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr status_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_position_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr odometry_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleLandDetected>::SharedPtr land_detected_sub_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr takeover_permission_sub_;
  rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr manual_yaw_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr clear_manual_yaw_sub_;
  rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr mode_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleAttitudeSetpoint>::SharedPtr setpoint_pub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Publisher<ControlDebug>::SharedPtr debug_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
}  // namespace

#ifndef PX4CTRL_UNIT_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<Px4Ctrl>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("px4ctrl"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
#endif
