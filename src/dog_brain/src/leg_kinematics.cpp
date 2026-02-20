#include "dog_brain/leg_kinematics.hpp"

#include <cmath>
#include <algorithm>

namespace dog_brain {

LegKinematics::LegKinematics(const RobotGeometry& geom)
    : geom_(geom)
{
}

LegJoints LegKinematics::solveIK(const FootPosition& foot, double /*side*/) const
{
    LegJoints joints;

    // 1. Hip roll — пока 0 (латеральное движение не реализовано)
    joints.hip = 0.0;

    // 2. Расстояние до стопы в сагиттальной плоскости
    //    L_HIP не входит — это латеральное смещение
    double d = std::sqrt(foot.x * foot.x + foot.z * foot.z);

    // Ограничиваем до рабочей зоны
    double max_len = geom_.l_thigh + geom_.l_shin - 0.001;
    double min_len = std::abs(geom_.l_thigh - geom_.l_shin) + 0.001;
    d = std::clamp(d, min_len, max_len);

    // 3. Колено — закон косинусов
    double cos_knee = (geom_.l_thigh * geom_.l_thigh +
                       geom_.l_shin * geom_.l_shin - d * d) /
                      (2.0 * geom_.l_thigh * geom_.l_shin);
    cos_knee = std::clamp(cos_knee, -1.0, 1.0);
    joints.knee = -(M_PI - std::acos(cos_knee));

    // 4. Бедро — atan2 для полного диапазона
    double cos_beta = (geom_.l_thigh * geom_.l_thigh +
                       d * d - geom_.l_shin * geom_.l_shin) /
                      (2.0 * geom_.l_thigh * d);
    cos_beta = std::clamp(cos_beta, -1.0, 1.0);
    joints.thigh = std::atan2(foot.x, -foot.z) + std::acos(cos_beta);

    return joints;
}

FootPosition LegKinematics::solveFK(const LegJoints& joints, double /*side*/) const
{
    FootPosition foot;

    // Прямая кинематика в сагиттальной плоскости
    // thigh_angle отсчитывается от -Z оси
    foot.x = geom_.l_thigh * std::sin(joints.thigh) +
             geom_.l_shin * std::sin(joints.thigh + joints.knee);

    foot.z = -(geom_.l_thigh * std::cos(joints.thigh) +
               geom_.l_shin * std::cos(joints.thigh + joints.knee));

    foot.y = 0.0;  // Латеральное — зарезервировано для hip_roll

    return foot;
}

double LegKinematics::maxReach() const
{
    return geom_.l_thigh + geom_.l_shin - 0.001;
}

double LegKinematics::minReach() const
{
    return std::abs(geom_.l_thigh - geom_.l_shin) + 0.001;
}

}  // namespace dog_brain
