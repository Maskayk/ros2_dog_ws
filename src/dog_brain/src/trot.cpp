#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using namespace std::chrono_literals;

class TrotNode : public rclcpp::Node
{
public:
    TrotNode() : Node("trot_node")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_position_controller/commands", 10);
        timer_ = this->create_wall_timer(
            20ms, std::bind(&TrotNode::timer_callback, this));
        start_time_ = this->now();
    }

private:
    const double L1 = 0.06;
    const double L2 = 0.144;
    const double L3 = 0.1525;

    // === ПАРАМЕТРЫ (Медленные и стабильные) ===
    const double WALKING_HEIGHT = -0.25;
    const double STEP_LENGTH = 0.05;
    const double STEP_HEIGHT = 0.04;
    const double PERIOD = 1.5;
    const double X_OFFSET = 0.02; // Чуть сдвигаем ноги вперед

    struct Point { double x; double y; double z; };

    std::vector<double> inverse_kinematics(double x, double y, double z)
    {
        double target_x = x + X_OFFSET;
        double theta1 = 0.0;
        
        // Работаем с абсолютными значениями
        double abs_z = std::abs(z);
        double dist_2d = std::sqrt(target_x * target_x + abs_z * abs_z);
        
        double max_reach = L2 + L3 - 0.005;
        if (dist_2d > max_reach) dist_2d = max_reach;

        double cos_knee = (L2 * L2 + L3 * L3 - dist_2d * dist_2d) / (2 * L2 * L3);
        if (cos_knee > 1.0) cos_knee = 1.0;
        if (cos_knee < -1.0) cos_knee = -1.0;
        double phi = std::acos(cos_knee);
        double theta3 = -(M_PI - phi); // Колено гнется назад

        double alpha = std::atan2(target_x, abs_z); // Внимание: abs_z
        double cos_beta = (L2 * L2 + dist_2d * dist_2d - L3 * L3) / (2 * L2 * dist_2d);
        if (cos_beta > 1.0) cos_beta = 1.0;
        if (cos_beta < -1.0) cos_beta = -1.0;
        double beta = std::acos(cos_beta);
        double theta2 = alpha + beta; // Бедро гнется вперед

        return {theta1, theta2, theta3};
    }

    Point get_leg_trajectory(double t, double phase_offset)
    {
        double cycle_t = std::fmod((t / PERIOD + phase_offset), 1.0);
        double x = 0.0;
        double z = WALKING_HEIGHT;

       if (cycle_t < 0.5)
        {
            // === SWING (Перенос) ===
            double swing_progress = cycle_t / 0.5;
            
            // X: БЫЛО -cos, СТАЛО +cos (Инверсия)
            x = (STEP_LENGTH / 2.0) * std::cos(M_PI * swing_progress);
            
            z = WALKING_HEIGHT + STEP_HEIGHT * std::sin(M_PI * swing_progress);
        }
        else
        {
            // === STANCE (Опора) ===
            double stance_progress = (cycle_t - 0.5) / 0.5;
            
            // X: БЫЛО (1 - 2p), СТАЛО (2p - 1) (Инверсия)
            // Теперь лапа едет Спереди -> Назад, толкая робота ВПЕРЕД.
            x = (STEP_LENGTH / 2.0) * (2.0 * stance_progress - 1.0);
            
            z = WALKING_HEIGHT; 
        }
        return {x, 0.0, z};
    }

    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        
        // Диагональ 1 (FL, RR)
        Point p1 = get_leg_trajectory(t, 0.0);
        // Диагональ 2 (FR, RL)
        Point p2 = get_leg_trajectory(t, 0.5);

        // ВАЖНО: Если лапы "двигаются как угодно", 
        // возможно, для задних лап нужен инвертированный X, 
        // так как их суставы могут быть развернуты на 180 градусов в URDF.
        // Пока считаем, что все суставы смотрят в одну сторону.

        auto fl = inverse_kinematics(p1.x, 0.0, p1.z);
        auto fr = inverse_kinematics(p2.x, 0.0, p2.z);
        auto rl = inverse_kinematics(p2.x, 0.0, p2.z);
        auto rr = inverse_kinematics(p1.x, 0.0, p1.z);

        auto msg = std_msgs::msg::Float64MultiArray();
        msg.data = {
            fl[0], fl[1], fl[2],
            fr[0], fr[1], fr[2],
            rl[0], rl[1], rl[2],
            rr[0], rr[1], rr[2]
        };
        publisher_->publish(msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Time start_time_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrotNode>());
    rclcpp::shutdown();
    return 0;
}