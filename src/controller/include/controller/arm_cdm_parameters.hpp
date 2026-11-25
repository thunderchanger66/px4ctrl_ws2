#pragma once
#include <Eigen/Dense>
#include <array>

namespace llc_mpc {

// Parameter order matches scripts/quad_x_model.py exactly.
constexpr int ARM_NP = 33;
using ArmStage = std::array<double, ARM_NP>;

struct ArmJointPrediction {
  Eigen::Vector3d q = Eigen::Vector3d::Zero();    // RAW Gazebo /joint_states convention
  Eigen::Vector3d dq = Eigen::Vector3d::Zero();
  Eigen::Vector3d ddq = Eigen::Vector3d::Zero();
};

// Coupled disturbance exerted by the manipulator on the UAV airframe.
// force_body  : F_dis   [N]    in body FLU frame (patent Eq. 19)
// moment_body : tau_dis [N*m]  in body FLU frame (patent Eq. 20)
struct CdmDisturbance {
  Eigen::Vector3d force_body = Eigen::Vector3d::Zero();
  Eigen::Vector3d moment_body = Eigen::Vector3d::Zero();
  bool valid = false;
};
struct ArmCdmGeometry {
  double total_mass = 0.0;                  // systemMass
  double arm_mass = 0.0;                    // manMass
  Eigen::Matrix3d uav_base_inertia = Eigen::Matrix3d::Identity();  // J_B, no arm
  // Joint frame offsets taken from the Gazebo model (SDF):
  //   joint_offset_12 : elbow joint origin in the arm_link1 frame
  //   joint_offset_23 : wrist joint origin in the arm_link2 frame
  Eigen::Vector3d joint_offset_12 = Eigen::Vector3d(0.198, 0.0005, -0.024);
  Eigen::Vector3d joint_offset_23 = Eigen::Vector3d(0.194, -0.0005, 0.0);
  Eigen::Vector3d link_mass = Eigen::Vector3d::Zero();
  std::array<Eigen::Vector3d,3> local_com{};
  std::array<Eigen::Matrix3d,3> local_com_inertia{};
  // Exactly the controller::setArmState convention: theta=[q1-pi,q2+pi,q3]
  Eigen::Vector3d angle_offset = Eigen::Vector3d(-3.14159265358979323846,
                                                   3.14159265358979323846,0.0);
};

class ArmCdmParameters {
public:
  explicit ArmCdmParameters(const ArmCdmGeometry &geometry);
  bool valid() const { return valid_; }
  // No UAV pose/velocity/omega needed; this runs OUTSIDE acados.
  bool calculate(const ArmJointPrediction& predicted, ArmStage& result) const;
  ArmStage neutral() const; // For UAV-only benchmark, preserves system M and J_B.

  // Coupled disturbance from a stage parameter set and the current UAV state.
  // The algebra is identical to the one embedded in the acados model
  // (scripts/quad_x_model.py), so the feed-forward term and the NMPC
  // prediction term describe the very same physical coupling.
  bool disturbance(const ArmStage& stage,
                   const Eigen::Vector3d& omega_body,
                   const Eigen::Vector3d& alpha_body,
                   const Eigen::Vector3d& velocity_body,
                   double total_thrust,
                   CdmDisturbance& out) const;
private:
  ArmCdmGeometry g_;
  bool valid_{false};
};

}  // namespace llc_mpc
