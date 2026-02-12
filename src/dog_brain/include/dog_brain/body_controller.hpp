#pragma once

#include "dog_brain/types.hpp"

namespace dog_brain {

struct StabilizationConfig {
    bool   enabled      = false;  // выключено по умолчанию (поведение как в оригинале)
    double kp_roll      = 0.0;
    double kp_pitch     = 0.0;
    double kd_roll      = 0.0;
    double kd_pitch     = 0.0;
    double max_correction_z = 0.02;  // макс. вертикальная коррекция (м)
    double max_correction_x = 0.02;  // макс. продольная коррекция (м)
};

class BodyController {
public:
    explicit BodyController(const StabilizationConfig& config = StabilizationConfig());

    /// Обновить ориентацию корпуса с IMU
    void updateOrientation(const BodyOrientation& orientation);

    /// Вычислить коррекции позиций стоп для всех 4 ног
    /// Возвращает дельты, которые добавляются к траекторным позициям
    QuadFootPos computeCorrections() const;

    void setConfig(const StabilizationConfig& config);
    const StabilizationConfig& config() const { return config_; }
    const BodyOrientation& orientation() const { return orientation_; }

    /// Преобразование кватерниона в углы Эйлера
    /// Точный порт из оригинального trot.cpp imu_callback
    static BodyOrientation quaternionToEuler(double qx, double qy, double qz, double qw);

private:
    StabilizationConfig config_;
    BodyOrientation orientation_;
};

}  // namespace dog_brain
