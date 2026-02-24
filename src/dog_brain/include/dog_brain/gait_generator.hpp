#pragma once

#include "dog_brain/types.hpp"
#include <array>

namespace dog_brain {

enum class LegState : uint8_t {
    SWING,
    STANCE
};

struct LegPhaseInfo {
    LegState state;
    double phase;       // 0.0-1.0 внутри текущей подфазы (swing или stance)
    double full_phase;  // 0.0-1.0 внутри полного цикла данной ноги
};

struct GaitConfig {
    double period      = 0.8;
    double duty_factor = 0.6;   // 60% stance, 40% swing (standard trot)

    // Фазовые смещения для каждой ноги [FL, FR, RL, RR]
    // Trot: диагональные пары синхронны
    std::array<double, LEG_COUNT> phase_offsets = {0.0, 0.5, 0.5, 0.0};
};

class GaitGenerator {
public:
    explicit GaitGenerator(const GaitConfig& config = GaitConfig());

    /// Обновить состояние по прошедшему времени
    std::array<LegPhaseInfo, LEG_COUNT> update(double elapsed_time) const;

    void setConfig(const GaitConfig& config);
    const GaitConfig& config() const { return config_; }

private:
    GaitConfig config_;
};

}  // namespace dog_brain
