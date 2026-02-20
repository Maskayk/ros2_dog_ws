#pragma once

#include "dog_brain/types.hpp"
#include "dog_brain/gait_generator.hpp"

namespace dog_brain {

struct TrajectoryConfig {
    double z_nominal   = -0.22;  // nominal standing height (m)
    double x_standing  = 0.03;   // FK foot.x offset for CoM compensation (m)
    double step_height = 0.04;   // max foot lift during swing (m)
    double step_amp_x  = 0.03;   // forward step amplitude scale
    double yaw_lever   = 0.08;   // yaw to x lever arm (m)
};

class FootTrajectory {
public:
    explicit FootTrajectory(const TrajectoryConfig& config = TrajectoryConfig());

    /// Вычислить позицию стопы для одной ноги
    FootPosition compute(const LegPhaseInfo& phase_info,
                         const VelocityCommand& vel,
                         LegId leg) const;

    /// Поза стойки (все ноги на номинальной высоте)
    FootPosition standingPose() const;

    void setConfig(const TrajectoryConfig& config);
    const TrajectoryConfig& config() const { return config_; }

private:
    TrajectoryConfig config_;

    FootPosition swingTrajectory(double phase, double total_x) const;
    FootPosition stanceTrajectory(double phase, double total_x) const;
};

}  // namespace dog_brain
