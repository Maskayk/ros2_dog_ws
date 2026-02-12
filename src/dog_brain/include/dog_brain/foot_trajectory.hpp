#pragma once

#include "dog_brain/types.hpp"
#include "dog_brain/gait_generator.hpp"

namespace dog_brain {

struct TrajectoryConfig {
    double z_nominal   = -0.22;  // номинальная высота стойки (м, совпадает с URDF init_pos)
    double step_height = 0.03;   // максимальный подъём стопы при swing (м)
    double step_amp_x  = 0.03;   // масштаб амплитуды шага вперёд
    double yaw_lever   = 0.08;   // плечо для преобразования yaw -> x (м)
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
