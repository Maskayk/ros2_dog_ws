  #pragma once
  #include <array>
  #include <cmath>

  class FootTrajectory{
      public:
      FootTrajectory(double step_length=0.08, double step_height=0.04, double duty=0.6) : step_length_(step_length), step_height_(step_height), duty_(duty) {}
    // t is phase [0,1)
    std::array<double,3> sample(double phase, double x0=0.0, double z0=-0.15) const {
    // forward/back in x, vertical in z
    double x = 0.0, z=z0;
    if(phase < duty_){
        // stance: move from +x0 to -x0 linearly
        double s = phase / duty_;
        x = x0 + ( -x0 - x0 ) * s; // from +x0 to -x0
        z = z0;
    }
    else{
            // swing: smooth curve (cycloid-like) from -x0 to +x0
        double s = (phase - duty_) / (1.0 - duty_); // 0..1
        // horizontal
        x = -x0 + 2.0*x0 * s;
        // vertical arc: use sine
        z = z0 + step_height_ * std::sin(M_PI * s);
    }
  return {x, 0.0, z}; // y is handled by gait/yaw

    }
    private:
    double step_length_;
    double step_height_;
    double duty_;

  };