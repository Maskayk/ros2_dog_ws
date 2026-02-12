#include "dog_brain/gait_generator.hpp"
#include <cmath>

namespace dog_brain {

GaitGenerator::GaitGenerator(const GaitConfig& config)
    : config_(config)
{
}

std::array<LegPhaseInfo, LEG_COUNT> GaitGenerator::update(double elapsed_time) const
{
    std::array<LegPhaseInfo, LEG_COUNT> result;

    // Глобальная фаза цикла [0, 1)
    double global_cycle = std::fmod(elapsed_time / config_.period, 1.0);
    if (global_cycle < 0.0) global_cycle += 1.0;

    double swing_duration = 1.0 - config_.duty_factor;  // 0.4 при duty=0.6

    for (int i = 0; i < LEG_COUNT; ++i) {
        // Фаза конкретной ноги с учётом смещения
        double leg_cycle = std::fmod(global_cycle + config_.phase_offsets[i], 1.0);

        result[i].full_phase = leg_cycle;

        // Конвенция: swing идёт первым в цикле (как в оригинальном trot.cpp)
        if (leg_cycle < swing_duration) {
            result[i].state = LegState::SWING;
            result[i].phase = leg_cycle / swing_duration;
        } else {
            result[i].state = LegState::STANCE;
            result[i].phase = (leg_cycle - swing_duration) / config_.duty_factor;
        }
    }

    return result;
}

void GaitGenerator::setConfig(const GaitConfig& config)
{
    config_ = config;
}

}  // namespace dog_brain
