#pragma once

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <Eigen/Core>

namespace controller
{
class PdMrac
{
public:
  using Vector3 = Eigen::Vector3d;
  using State = Eigen::Matrix<double, 6, 1>;
  using Gain1 = Eigen::Matrix<double, 3, 6>;
  using Gain2 = Eigen::Matrix<double, 3, 7>;
  using Regressor = Eigen::Matrix<double, 7, 1>;

  struct Parameters
  {
    Vector3 kp{1.5, 1.5, 1.8};
    Vector3 kv{1.2, 1.2, 1.4};
    double rho{4.5};
    double gamma1{0.25};
    double gamma2{0.25};
    double alpha{0.3};
    double beta{5.0};
    double kappa1{0.02};
    double kappa2{0.02};
    double velocity_scale_mps{2.0};
    double gain1_element_limit{2.0};
    double gain2_element_limit{1.0};
    double gain1_norm_limit{8.0};
    double gain2_norm_limit{4.0};
    double disturbance_limit_mps2{2.0};
  };

  struct Output
  {
    Vector3 acceleration{Vector3::Zero()};
    Vector3 estimated_disturbance{Vector3::Zero()};
    Vector3 model_error_position{Vector3::Zero()};
    Vector3 model_error_velocity{Vector3::Zero()};
    double gain1_norm{0.0};
    double gain2_norm{0.0};
    double filter_norm{0.0};
  };

  explicit PdMrac(const Parameters & parameters)
  : parameters_(parameters)
  {
    if (!parameters_.kp.allFinite() || !parameters_.kv.allFinite() ||
      (parameters_.kp.array() <= 0.0).any() || (parameters_.kv.array() <= 0.0).any() ||
      !std::isfinite(parameters_.rho) || parameters_.rho <= 0.0 ||
      !std::isfinite(parameters_.gamma1) || parameters_.gamma1 <= 0.0 ||
      !std::isfinite(parameters_.gamma2) || parameters_.gamma2 <= 0.0 ||
      !std::isfinite(parameters_.alpha) || parameters_.alpha < 0.0 ||
      !std::isfinite(parameters_.beta) || parameters_.beta <= 0.0 ||
      !std::isfinite(parameters_.kappa1) || parameters_.kappa1 < 0.0 ||
      !std::isfinite(parameters_.kappa2) || parameters_.kappa2 < 0.0 ||
      !std::isfinite(parameters_.velocity_scale_mps) || parameters_.velocity_scale_mps <= 0.0 ||
      !std::isfinite(parameters_.gain1_element_limit) || parameters_.gain1_element_limit <= 0.0 ||
      !std::isfinite(parameters_.gain2_element_limit) || parameters_.gain2_element_limit <= 0.0 ||
      !std::isfinite(parameters_.gain1_norm_limit) || parameters_.gain1_norm_limit <= 0.0 ||
      !std::isfinite(parameters_.gain2_norm_limit) || parameters_.gain2_norm_limit <= 0.0 ||
      !std::isfinite(parameters_.disturbance_limit_mps2) ||
      parameters_.disturbance_limit_mps2 <= 0.0)
    {
      throw std::invalid_argument("invalid PD-MRAC parameters");
    }
    reset(Vector3::Zero(), Vector3::Zero());
  }

  void reset(const Vector3 & position, const Vector3 & velocity)
  {
    if (!position.allFinite() || !velocity.allFinite()) {
      throw std::invalid_argument("PD-MRAC reset state must be finite");
    }
    gain1_.setZero();
    gain2_.setZero();
    filtered_error_.setZero();
  }

  Output update(
    const Vector3 & position_reference,
    const Vector3 & velocity_reference,
    const Vector3 & acceleration_reference,
    const Vector3 & position,
    const Vector3 & velocity,
    double dt,
    bool adaptation_enabled,
    double feedback_scale = 1.0)
  {
    Output output;
    if (!position_reference.allFinite() || !velocity_reference.allFinite() ||
      !acceleration_reference.allFinite() || !position.allFinite() ||
      !velocity.allFinite() || !std::isfinite(dt) ||
      !std::isfinite(feedback_scale) || feedback_scale <= 0.0)
    {
      return output;
    }
    dt = std::clamp(dt, 0.0, 0.05);

    State error;
    error.head<3>() = position - position_reference;
    error.tail<3>() = velocity - velocity_reference;
    const State error_rate = -parameters_.beta * filtered_error_ + error;
    if (dt > 0.0) filtered_error_ += error_rate * dt;

    Vector3 lyapunov_error;
    for (int axis = 0; axis < 3; ++axis) {
      const double p12 = 1.0 / (2.0 * parameters_.kp(axis));
      const double p22 = (1.0 + 1.0 / parameters_.kp(axis)) /
        (2.0 * (2.0 * parameters_.kp(axis) / parameters_.kv(axis)));
      lyapunov_error(axis) = p12 * error(axis) + p22 * error(axis + 3);
    }

    const State trend_error = error + parameters_.alpha * parameters_.beta * error_rate;
    Vector3 trend_lyapunov_error;
    for (int axis = 0; axis < 3; ++axis) {
      const double p12 = 1.0 / (2.0 * parameters_.kp(axis));
      const double p22 = (1.0 + 1.0 / parameters_.kp(axis)) /
        (2.0 * (2.0 * parameters_.kp(axis) / parameters_.kv(axis)));
      trend_lyapunov_error(axis) =
        p12 * trend_error(axis) + p22 * trend_error(axis + 3);
    }

    Regressor phi;
    const double velocity_scale = parameters_.velocity_scale_mps;
    for (int axis = 0; axis < 3; ++axis) {
      const double scaled_velocity = velocity(axis) / velocity_scale;
      phi(axis + 1) = scaled_velocity;
      phi(axis + 4) = std::abs(scaled_velocity) * scaled_velocity;
    }
    phi(0) = 1.0;

    if (adaptation_enabled && dt > 0.0) {
      gain1_ += (-trend_lyapunov_error * error.transpose() / parameters_.gamma1 -
        parameters_.kappa1 * gain1_) * dt;
      gain2_ += (-trend_lyapunov_error * phi.transpose() / parameters_.gamma2 -
        parameters_.kappa2 * gain2_) * dt;
      gain1_ = gain1_.unaryExpr([this](double value) {
          return std::clamp(value, -parameters_.gain1_element_limit,
            parameters_.gain1_element_limit);
        });
      gain2_ = gain2_.unaryExpr([this](double value) {
          return std::clamp(value, -parameters_.gain2_element_limit,
            parameters_.gain2_element_limit);
        });
      project_norm(gain1_, parameters_.gain1_norm_limit);
      project_norm(gain2_, parameters_.gain2_norm_limit);
    }

    output.estimated_disturbance = gain2_ * phi;
    const double disturbance_norm = output.estimated_disturbance.norm();
    if (disturbance_norm > parameters_.disturbance_limit_mps2) {
      output.estimated_disturbance *= parameters_.disturbance_limit_mps2 / disturbance_norm;
    }
    output.acceleration = acceleration_reference -
      feedback_scale * parameters_.rho * lyapunov_error +
      gain1_ * error + output.estimated_disturbance;
    output.model_error_position = error.head<3>();
    output.model_error_velocity = error.tail<3>();
    output.gain1_norm = gain1_.norm();
    output.gain2_norm = gain2_.norm();
    output.filter_norm = filtered_error_.norm();
    if (!output.acceleration.allFinite() || !output.estimated_disturbance.allFinite()) {
      reset(position, velocity);
      output = Output{};
    }
    return output;
  }

private:
  template<typename Matrix>
  static void project_norm(Matrix & matrix, double limit)
  {
    const double norm = matrix.norm();
    if (norm > limit) matrix *= limit / norm;
  }

  Parameters parameters_;
  Gain1 gain1_{Gain1::Zero()};
  Gain2 gain2_{Gain2::Zero()};
  State filtered_error_{State::Zero()};
};
}  // namespace controller
