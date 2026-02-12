#include "dog_brain/foot_trajectory.hpp"
#include <cmath>

namespace dog_brain {

FootTrajectory::FootTrajectory(const TrajectoryConfig& config)
    : config_(config)
{
}

FootPosition FootTrajectory::compute(const LegPhaseInfo& phase_info,
                                     const VelocityCommand& vel,
                                     LegId leg) const
{
    int idx = static_cast<int>(leg);

    // Вклад поворота yaw в продольное смещение (из оригинала: -vel_yaw * signs_y[i] * 0.1)
    double rot_x = -vel.vyaw * LEG_SIGN_Y[idx] * config_.yaw_lever;
    double total_x = (vel.vx * config_.step_amp_x) + rot_x;

    if (phase_info.state == LegState::SWING) {
        return swingTrajectory(phase_info.phase, total_x);
    } else {
        return stanceTrajectory(phase_info.phase, total_x);
    }
}

FootPosition FootTrajectory::standingPose() const
{
    return {0.0, 0.0, config_.z_nominal};
}

FootPosition FootTrajectory::swingTrajectory(double p, double total_x) const
{
    FootPosition foot;

    // Smoothstep для X: 3p² - 2p³, отображаем на [-total_x, +total_x]
    double smooth_p = 3.0 * p * p - 2.0 * p * p * p;
    foot.x = total_x * (2.0 * smooth_p - 1.0);

    // Полиномиальный подъём для Z: 16p²(1-p)² — колокол с нулевой скоростью на концах
    foot.z = config_.z_nominal + config_.step_height * 16.0 * p * p * (1.0 - p) * (1.0 - p);

    foot.y = 0.0;

    return foot;
}

FootPosition FootTrajectory::stanceTrajectory(double p, double total_x) const
{
    FootPosition foot;

    // Линейный push-back: от +total_x к -total_x
    foot.x = total_x * (1.0 - 2.0 * p);
    foot.z = config_.z_nominal;
    foot.y = 0.0;

    return foot;
}

void FootTrajectory::setConfig(const TrajectoryConfig& config)
{
    config_ = config;
}

}  // namespace dog_brain
