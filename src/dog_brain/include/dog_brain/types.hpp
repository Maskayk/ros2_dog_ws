#pragma once

#include <array>
#include <cstdint>

namespace dog_brain {

// ---- Идентификация ног ----
// Порядок совпадает с controllers.yaml:
// FL_hip, FL_thigh, FL_shin, FR_..., RL_..., RR_...
enum class LegId : uint8_t {
    FL = 0,
    FR = 1,
    RL = 2,
    RR = 3
};
constexpr int LEG_COUNT = 4;

// ---- Геометрия робота (из dog.urdf.xacro) ----
struct RobotGeometry {
    double l_hip        = 0.06;    // hip link lateral length (m)
    double l_thigh      = 0.144;   // thigh link length (m)
    double l_shin       = 0.1525;  // shin link length (m)
    double trunk_length = 0.475;   // body length (m)
    double trunk_width  = 0.18;    // body width (m)
    double trunk_height = 0.1;     // body height (m)
};

inline const RobotGeometry& defaultGeometry()
{
    static const RobotGeometry geom;
    return geom;
}

// ---- Углы суставов одной ноги ----
struct LegJoints {
    double hip   = 0.0;  // roll (X axis)
    double thigh = 0.0;  // pitch (Y axis)
    double knee  = 0.0;  // pitch (Y axis)
};

// ---- Позиция стопы в системе координат бедра ----
struct FootPosition {
    double x = 0.0;  // вперёд (+) / назад (-)
    double y = 0.0;  // влево (+) / вправо (-), пока не используется
    double z = 0.0;  // вниз (отрицательные значения)
};

// ---- Ориентация корпуса (от IMU) ----
struct BodyOrientation {
    double roll  = 0.0;
    double pitch = 0.0;
    double yaw   = 0.0;
};

// ---- Команда скорости ----
struct VelocityCommand {
    double vx   = 0.0;  // м/с вперёд
    double vy   = 0.0;  // м/с влево
    double vyaw = 0.0;  // рад/с поворот
};

// ---- Массивы для 4 ног ----
using QuadJoints  = std::array<LegJoints, LEG_COUNT>;
using QuadFootPos = std::array<FootPosition, LEG_COUNT>;

// ---- Знаки позиций ног относительно центра корпуса ----
// FL(+X,+Y), FR(+X,-Y), RL(-X,+Y), RR(-X,-Y)
constexpr double LEG_SIGN_X[LEG_COUNT] = { 1.0,  1.0, -1.0, -1.0};
constexpr double LEG_SIGN_Y[LEG_COUNT] = { 1.0, -1.0,  1.0, -1.0};

// ---- Константы ----
constexpr int JOINTS_PER_LEG = 3;
constexpr int TOTAL_JOINTS   = LEG_COUNT * JOINTS_PER_LEG;  // 12

}  // namespace dog_brain
