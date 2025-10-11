#include "dog_controllers/leg_kinematics.hpp"
#include <cmath>
#include <algorithm>

LegKinematics3DOF::LegKinematics3DOF(double L1, double L2)
: L1_(L1), L2_(L2)
{
  if (L1_ <= 0 || L2_ <= 0)
    throw std::runtime_error("Invalid link lengths");
}

std::array<double, 3> LegKinematics3DOF::inverse(const std::array<double, 3>& p) const
{
  double x = p[0], y = p[1], z = p[2];
  double yaw = std::atan2(y, x);

  double r = std::hypot(x, y);
  double x2 = r;
  double z2 = z;

  double d = std::hypot(x2, z2);
  double maxd = L1_ + L2_;

  // защита от недостижимых положений
  if (d > maxd)
  {
    double scale = maxd / d;
    x2 *= scale;
    z2 *= scale;
    d = maxd;
  }

  if (d < 1e-8) d = 1e-8;

  double alpha = std::atan2(z2, x2);
  double cos_beta = (L1_*L1_ + d*d - L2_*L2_) / (2.0 * L1_ * d);
  cos_beta = std::clamp(cos_beta, -1.0, 1.0);
  double beta = std::acos(cos_beta);
  double hip = alpha - beta;

  double cos_k = (L1_*L1_ + L2_*L2_ - d*d) / (2.0 * L1_ * L2_);
  cos_k = std::clamp(cos_k, -1.0, 1.0);
  double knee = M_PI - std::acos(cos_k);  // ← тут у тебя раньше была ошибка с минусом!

  return {yaw, hip, knee};
}

std::array<std::array<double,3>, 3> LegKinematics3DOF::jacobian(const std::array<double,3>& q) const
{
  double yaw = q[0], hip = q[1], knee = q[2];
  double knee_internal = M_PI - knee;
  double a1 = hip;
  double a2 = hip + knee_internal;

  double dx_dhip = -L1_*std::sin(a1) - L2_*std::sin(a2);
  double dz_dhip =  L1_*std::cos(a1) + L2_*std::cos(a2);

  double dx_dknee =  L2_*std::sin(a2);
  double dz_dknee = -L2_*std::cos(a2);

  double x2 = L1_*std::cos(a1) + L2_*std::cos(a2);

  double cos_y = std::cos(yaw), sin_y = std::sin(yaw);

  std::array<double,3> dp_dyaw  = {-sin_y * x2, cos_y * x2, 0.0};
  std::array<double,3> dp_dhip  = {cos_y * dx_dhip, sin_y * dx_dhip, dz_dhip};
  std::array<double,3> dp_dknee = {cos_y * dx_dknee, sin_y * dx_dknee, dz_dknee};

  return {dp_dyaw, dp_dhip, dp_dknee};
}

double LegKinematics3DOF::clamp(double v, double lo, double hi)
{
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

double LegKinematics3DOF::L1() const { return L1_; }
double LegKinematics3DOF::L2() const { return L2_; }
