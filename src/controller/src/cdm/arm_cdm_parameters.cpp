#include "controller/arm_cdm_parameters.hpp"
#include <cmath>

namespace llc_mpc {
namespace {
using V = Eigen::Vector3d;
using M3 = Eigen::Matrix3d;
const V EY = V::UnitY();

M3 rotY(double th) {
  const double c = std::cos(th), s = std::sin(th);
  M3 R;
  R << c,0,s, 0,1,0, -s,0,c;
  return R;
}
M3 skew(const V& x) {
  M3 S;
  S << 0,-x.z(),x.y(), x.z(),0,-x.x(), -x.y(),x.x(),0;
  return S;
}
void packSym(const M3& A, ArmStage& out, int i) {
  // [xx, xy, xz, yy, yz, zz]
  out[i]   = A(0,0);
  out[i+1] = A(0,1);
  out[i+2] = A(0,2);
  out[i+3] = A(1,1);
  out[i+4] = A(1,2);
  out[i+5] = A(2,2);
}
void putVec(const V& v, ArmStage& out, int i) {
  for (int k=0; k<3; ++k) out[i+k]=v(k);
}
M3 unSym(const ArmStage& s, int i) {
  M3 A;
  A << s[i],   s[i+1], s[i+2],
       s[i+1], s[i+3], s[i+4],
       s[i+2], s[i+4], s[i+5];
  return A;
}
bool finite(const ArmStage& v) {
  for (double x : v) if (!std::isfinite(x)) return false;
  return true;
}
} // namespace

ArmCdmParameters::ArmCdmParameters(const ArmCdmGeometry& geometry) : g_(geometry) {
  if (g_.arm_mass <= 0.0 || g_.total_mass <= g_.arm_mass
      || !g_.uav_base_inertia.allFinite() || !g_.link_mass.allFinite()
      || !g_.joint_offset_12.allFinite() || !g_.joint_offset_23.allFinite()
      || !g_.angle_offset.allFinite()) return;
  double measured_arm_mass = 0.0;
  for (int k=0;k<3;++k) {
    if (g_.link_mass(k) <= 0.0 || !g_.local_com[k].allFinite()
        || !g_.local_com_inertia[k].allFinite()) return;
    measured_arm_mass += g_.link_mass(k);
  }
  if (std::abs(measured_arm_mass-g_.arm_mass)>1e-4) return;
  Eigen::LLT<M3> base(g_.uav_base_inertia);
  valid_ = base.info()==Eigen::Success;
}

ArmStage ArmCdmParameters::neutral() const {
  ArmStage out{};
  packSym(g_.uav_base_inertia,out,9);
  const M3 K = g_.uav_base_inertia.inverse();
  packSym(K,out,21);
  return out;
}

bool ArmCdmParameters::calculate(const ArmJointPrediction& predicted,
                                  ArmStage& result) const {
  if (!valid_ || !predicted.q.allFinite()
      || !predicted.dq.allFinite() || !predicted.ddq.allFinite()) return false;

  const V q = predicted.q + g_.angle_offset; // source controller.h convention
  const V dq = predicted.dq, ddq=predicted.ddq;
  const std::array<double,3> th{
    q(0), q(0)+q(1), q(0)+q(1)+q(2)
  };
  const std::array<double,3> dth{
    dq(0), dq(0)+dq(1), dq(0)+dq(1)+dq(2)
  };
  const std::array<double,3> ddth{
    ddq(0), ddq(0)+ddq(1), ddq(0)+ddq(1)+ddq(2)
  };
  std::array<M3,3> R{rotY(th[0]),rotY(th[1]),rotY(th[2])};
  const V mount(0.0,-0.019,0.0405);
  const V offset12 = g_.joint_offset_12;
  const V offset23 = g_.joint_offset_23;

  std::array<V,3> com, vel, acc;
  // Each p_i = mount + sum_j Ry(sum_{l<=j}q_l) * fixed_vector_j.
  // 0.024m is along local Y, NOT Z. Its rotational derivative is zero.
  for (int i=0;i<3;++i) {
    com[i]=mount;
    vel[i].setZero();
    acc[i].setZero();
    for (int j=0;j<=i;++j) {
      V local=V::Zero();
      if (j==0 && i>=1) local+=offset12;
      if (j==1 && i>=2) local+=offset23;
      if (j==i) local+=g_.local_com[i];

      const V segment = R[j]*local;
      const V dseg = EY.cross(segment);
      const V ddseg = EY.cross(dseg);
      com[i] += segment;
      vel[i] += dth[j]*dseg;
      acc[i] += ddth[j]*dseg + dth[j]*dth[j]*ddseg;
    }
  }

  V sumP=V::Zero(), sumV=V::Zero(), sumA=V::Zero();
  M3 Ia=M3::Zero(), Idot=M3::Zero(), eye=M3::Identity();
  for (int i=0;i<3;++i) {
    const double mi=g_.link_mass(i);
    sumP+=mi*com[i];
    sumV+=mi*vel[i];
    sumA+=mi*acc[i];
    const M3 Ii=R[i]*g_.local_com_inertia[i]*R[i].transpose();
    const M3 Wi=skew(V(0.0,dth[i],0.0));
    const M3 IiDot=Wi*Ii-Ii*Wi;
    Ia += Ii + mi*(com[i].squaredNorm()*eye-com[i]*com[i].transpose());
    Idot += IiDot + mi*(2.0*com[i].dot(vel[i])*eye
                       -vel[i]*com[i].transpose()-com[i]*vel[i].transpose());
  }
  // romc=(sum m_i*r_i)/m_arm, independently of M_total.
  const V r=sumP/g_.arm_mass;
  const V dr=sumV/g_.arm_mass;
  const V ddr=sumA/g_.arm_mass;
  const M3 J=g_.uav_base_inertia+Ia;
  const M3 S=skew(r);
  const M3 H=J+(g_.arm_mass*g_.arm_mass/g_.total_mass)*S*S;
  Eigen::LLT<M3> llt(H);
  if (llt.info()!=Eigen::Success) return false;
  const M3 K=llt.solve(eye);
  if (!K.allFinite()) return false;

  result.fill(0.0);
  putVec(r,result,0);
  putVec(dr,result,3);
  putVec(ddr,result,6);
  packSym(J,result,9);
  packSym(Idot,result,15);
  packSym(K,result,21);
  putVec(r.cross(dr),result,27);
  putVec(r.cross(ddr),result,30);
  return finite(result);
}

bool ArmCdmParameters::disturbance(const ArmStage& stage,
                                    const Eigen::Vector3d& omega_body,
                                    const Eigen::Vector3d& alpha_body,
                                    const Eigen::Vector3d& velocity_body,
                                    double total_thrust,
                                    CdmDisturbance& out) const {
  out = CdmDisturbance{};
  if (!valid_) return false;

  const V r(stage[0],stage[1],stage[2]);
  const V rdot(stage[3],stage[4],stage[5]);
  const V rddot(stage[6],stage[7],stage[8]);
  const M3 J = unSym(stage,9);
  const M3 Idot = unSym(stage,15);
  const V h1(stage[27],stage[28],stage[29]);
  const V h2(stage[30],stage[31],stage[32]);

  if (!r.allFinite() || !rdot.allFinite() || !rddot.allFinite() ||
      !J.allFinite() || !Idot.allFinite() ||
      !omega_body.allFinite() || !alpha_body.allFinite() ||
      !velocity_body.allFinite()) return false;

  // c_force = w x (w x r) + 2 w x rdot + rddot   (apparent force of the arm CoM)
  const V c_force = omega_body.cross(omega_body.cross(r))
                  + 2.0*omega_body.cross(rdot) + rddot;
  // c_torque = vB x (w x r) + w x (r x rdot) + r x rddot
  const V c_torque = velocity_body.cross(omega_body.cross(r))
                   + omega_body.cross(h1) + h2;

  const M3 S = skew(r);
  const M3 I_arm = J - g_.uav_base_inertia;

  // Patent Eq. (19): F_dis = -m_arm * ( w x (w x r) + alpha x r + 2 w x rdot + rddot )
  const V force = -g_.arm_mass * (c_force + alpha_body.cross(r));

  // Patent Eq. (20) written in the algebra of the implemented acados model:
  // the arm reaction moment that the airframe has to compensate.
  const V thrust_e3(0.0,0.0,total_thrust);
  const V moment = I_arm*alpha_body
                 + omega_body.cross(I_arm*omega_body)
                 + Idot*omega_body
                 + g_.arm_mass*c_torque
                 + (g_.arm_mass/g_.total_mass)*(S*(thrust_e3 - g_.arm_mass*c_force));

  if (!force.allFinite() || !moment.allFinite()) return false;
  out.force_body = force;
  out.moment_body = moment;
  out.valid = true;
  return true;
}
} // namespace llc_mpc
