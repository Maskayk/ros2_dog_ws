#include "dog_brain/body_controller.hpp"
#include <cmath>
#include <algorithm>

namespace dog_brain {

BodyController::BodyController(const StabilizationConfig& config)
    : config_(config)
{
}

void BodyController::updateOrientation(const BodyOrientation& orientation)
{
    orientation_ = orientation;
}

QuadFootPos BodyController::computeCorrections() const
{
    QuadFootPos corrections{};

    if (!config_.enabled) {
        return corrections;  // нулевые коррекции, поведение как в оригинале
    }

    // Пропорциональная стабилизация:
    // Roll: левые ноги вверх/вниз, правые — наоборот
    // Pitch: передние ноги вверх/вниз, задние — наоборот
    for (int i = 0; i < LEG_COUNT; ++i) {
        // Roll коррекция по Z:
        // Если корпус наклонён вправо (roll < 0), левые ноги опускаем, правые поднимаем
        double dz_roll = -config_.kp_roll * orientation_.roll * LEG_SIGN_Y[i];
        dz_roll = std::clamp(dz_roll, -config_.max_correction_z, config_.max_correction_z);

        // Pitch коррекция по Z:
        // Если корпус наклонён вперёд (pitch > 0), передние ноги поднимаем, задние опускаем
        double dz_pitch = config_.kp_pitch * orientation_.pitch * LEG_SIGN_X[i];
        dz_pitch = std::clamp(dz_pitch, -config_.max_correction_z, config_.max_correction_z);

        corrections[i].z = dz_roll + dz_pitch;
        corrections[i].x = 0.0;
        corrections[i].y = 0.0;
    }

    return corrections;
}

void BodyController::setConfig(const StabilizationConfig& config)
{
    config_ = config;
}

BodyOrientation BodyController::quaternionToEuler(double qx, double qy, double qz, double qw)
{
    BodyOrientation orient;

    // Roll (вращение вокруг X)
    double sinr_cosp = 2.0 * (qw * qx + qy * qz);
    double cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy);
    orient.roll = std::atan2(sinr_cosp, cosr_cosp);

    // Pitch (вращение вокруг Y)
    double sinp = 2.0 * (qw * qy - qz * qx);
    if (std::abs(sinp) >= 1.0) {
        orient.pitch = std::copysign(M_PI / 2.0, sinp);  // gimbal lock
    } else {
        orient.pitch = std::asin(sinp);
    }

    // Yaw (вращение вокруг Z) — пока не используется, но вычисляем
    double siny_cosp = 2.0 * (qw * qz + qx * qy);
    double cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz);
    orient.yaw = std::atan2(siny_cosp, cosy_cosp);

    return orient;
}

}  // namespace dog_brain
