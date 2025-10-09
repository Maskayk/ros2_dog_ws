#pragma once
#include <cmath>
#include <array>

class LegKinematics3DOF
{
public:
  LegKinematics3DOF(double L1, double L2)
  : L1_(L1), L2_(L2) {}

  std::array<double, 3> inverse(double x, double y, double z)
  {
    double yaw = atan2(y, x);
    double xy = sqrt(x*x + y*y);
    double d = sqrt(xy*xy + z*z);
    if (d > (L1_ + L2_)) d = L1_ + L2_;

    double alpha = atan2(z, xy);
    double beta = acos((L1_*L1_ + d*d - L2_*L2_) / (2*L1_*d));
    double hip = alpha - beta;
    double knee = M_PI - acos((L1_*L1_ + L2_*L2_ - d*d) / (2*L1_*L2_));

    return {yaw, hip, knee};
  }

private:
  double L1_, L2_;
};
