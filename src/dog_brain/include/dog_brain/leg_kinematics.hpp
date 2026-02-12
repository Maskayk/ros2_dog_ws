#pragma once

#include "dog_brain/types.hpp"

namespace dog_brain {

class LegKinematics {
public:
    explicit LegKinematics(const RobotGeometry& geom = defaultGeometry());

    /// Обратная кинематика: позиция стопы (в СК бедра) -> углы суставов
    /// side: +1.0 для левых ног, -1.0 для правых (влияет на hip_roll)
    LegJoints solveIK(const FootPosition& foot, double side = 1.0) const;

    /// Прямая кинематика: углы суставов -> позиция стопы (в СК бедра)
    FootPosition solveFK(const LegJoints& joints, double side = 1.0) const;

    double maxReach() const;
    double minReach() const;

    const RobotGeometry& geometry() const { return geom_; }

private:
    RobotGeometry geom_;
};

}  // namespace dog_brain
