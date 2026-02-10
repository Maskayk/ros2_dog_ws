#include <chrono>
#include <cmath>
#include <memory>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp" 

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
        
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&TrotNode::cmd_vel_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Ready for Teleop! Use /cmd_vel");
    }

private:
    const double L1 = 0.06;
    const double L2 = 0.144;
    const double L3 = 0.1525;
    
    // Целевые скорости
    double target_linear_x_ = 0.0;
    double target_angular_z_ = 0.0;

    // Подписчик
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;

    // === ПАРАМЕТРЫ ===
    // === НАСТРОЙКИ "ТАНК" ===
    const double WALKING_HEIGHT = -0.22; // Ползаем на брюхе (макс стабильность)
    const double STEP_LENGTH = 0.02;     // 2 сантиметра! Почти на месте.
    const double STEP_HEIGHT = 0.04;
    const double PERIOD = 1.0;
    const double X_OFFSET = 0.0;         // Без хитростей

    struct Point { double x; double y; double z; };

    std::vector<double> inverse_kinematics(double x, double y, double z)
    {
        double target_x = x + X_OFFSET;
        double theta1 = 0.0;
        
        double abs_z = std::abs(z);
        double dist_2d = std::sqrt(target_x * target_x + abs_z * abs_z);
        
        double max_reach = L2 + L3 - 0.005;
        if (dist_2d > max_reach) dist_2d = max_reach;

        double cos_knee = (L2 * L2 + L3 * L3 - dist_2d * dist_2d) / (2 * L2 * L3);
        if (cos_knee > 1.0) cos_knee = 1.0;
        if (cos_knee < -1.0) cos_knee = -1.0;
        double phi = std::acos(cos_knee);
        double theta3 = -(M_PI - phi); 

        double alpha = std::atan2(target_x, abs_z); 
        double cos_beta = (L2 * L2 + dist_2d * dist_2d - L3 * L3) / (2 * L2 * dist_2d);
        if (cos_beta > 1.0) cos_beta = 1.0;
        if (cos_beta < -1.0) cos_beta = -1.0;
        double beta = std::acos(cos_beta);
        double theta2 = alpha + beta; 

        return {theta1, theta2, theta3};
    }

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
        target_linear_x_ = msg->linear.x;
        target_angular_z_ = msg->angular.z;
    }

    Point get_leg_trajectory(double t, double phase_offset)
    {
        double cycle_t = std::fmod((t / PERIOD + phase_offset), 1.0);
        double x = 0.0;
        double z = WALKING_HEIGHT;
        
        // Масштабируем длину шага от скорости
        double current_step_length = STEP_LENGTH * target_linear_x_; 

       if (std::abs(current_step_length) < 0.001) {
         // СТОИМ НА МЕСТЕ
         // Вместо просто высоты, давай опустим его на 1 см ниже.
         // Это создаст сильное давление на стопы и остановит дрейф.
         return {0.0, 0.0, WALKING_HEIGHT - 0.01}; 
    }
        if (cycle_t < 0.5)
        {
            // === SWING (Перенос) ===
            double swing_progress = cycle_t / 0.5; // <--- ВОТ ЭТОЙ СТРОКИ НЕ БЫЛО
            
            // X: Двигаемся ВПЕРЕД (-cos идет от -1 к 1)
            x = (current_step_length / 2.0) * -std::cos(M_PI * swing_progress);
            
            z = WALKING_HEIGHT + STEP_HEIGHT * std::sin(M_PI * swing_progress);
        }
        
        else
        {
            // === STANCE (Опора) ===
            double stance_progress = (cycle_t - 0.5) / 0.5; // <--- И ЭТОЙ НЕ БЫЛО
            
            // X: Толкаем землю НАЗАД (линейно от 1 к -1)
            x = (current_step_length / 2.0) * (1.0 - 2.0 * stance_progress); 
            z = WALKING_HEIGHT;
        }
        
        return {x, 0.0, z};
    }
    void timer_callback()
    {
        double t = (this->now() - start_time_).seconds();
        
        Point p1 = get_leg_trajectory(t, 0.0);
        Point p2 = get_leg_trajectory(t, 0.5);

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