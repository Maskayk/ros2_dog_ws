#pragma once
#include <array>
#include <cmath>
#include <stdexcept>

struct leg_limits
{
  double yaw_min, yaw_max;
  double hip_min, hip_max;
  double knee_min, knee_max;
};

class LegKinematics3DOF
{
public:
  LegKinematics3DOF(double L1, double L2);

  std::array<double, 3> inverse(const std::array<double, 3>& p) const;

  std::array<std::array<double,3>, 3> jacobian(const std::array<double,3>& q) const;

  static double clamp(double v, double lo, double hi);

  double L1() const;
  double L2() const;

private:
  double L1_, L2_;
};
