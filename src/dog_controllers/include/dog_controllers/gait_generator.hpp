#pragma once
#include <vector>
#include <cmath>

namespace dog_controllers {

class GaitGenerator {
public:
  GaitGenerator(double step_time = 0.8) : step_time_(step_time) {}
  double phase(double time) const { return fmod(time / step_time_, 1.0); }

private:
  double step_time_;
};

}  // namespace dog_controllers
