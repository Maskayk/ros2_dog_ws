#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include "dog_brain/types.hpp"
#include "dog_brain/leg_kinematics.hpp"
#include "dog_brain/gait_generator.hpp"
#include "dog_brain/foot_trajectory.hpp"
#include "dog_brain/body_controller.hpp"

using namespace std::chrono_literals;

namespace dog_brain {

class TrotNode : public rclcpp::Node
{
public:
    TrotNode() : Node("trot_node")
    {
        // --- Параметры походки ---
        this->declare_parameter("gait.period", 0.6);
        this->declare_parameter("gait.duty_factor", 0.65);

        // --- Параметры траектории ---
        this->declare_parameter("trajectory.z_nominal", -0.22);
        this->declare_parameter("trajectory.step_height", 0.03);
        this->declare_parameter("trajectory.step_amp_x", 0.03);
        this->declare_parameter("trajectory.yaw_lever", 0.08);

        // --- Параметры стабилизации ---
        this->declare_parameter("stabilization.enabled", false);
        this->declare_parameter("stabilization.kp_roll", 0.0);
        this->declare_parameter("stabilization.kp_pitch", 0.0);
        this->declare_parameter("stabilization.kd_roll", 0.0);
        this->declare_parameter("stabilization.kd_pitch", 0.0);
        this->declare_parameter("stabilization.max_correction_z", 0.02);
        this->declare_parameter("stabilization.max_correction_x", 0.02);

        // --- Параметры фильтра ---
        this->declare_parameter("filter.alpha", 0.15);
        this->declare_parameter("filter.stationary_threshold", 0.01);

        // Загружаем параметры в компоненты
        loadParameters();

        // --- ROS интерфейсы ---
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
            "/joint_group_position_controller/commands", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&TrotNode::cmdVelCallback, this, std::placeholders::_1));

        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&TrotNode::imuCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(
            20ms, std::bind(&TrotNode::timerCallback, this));

        start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
            "TrotNode started (modular architecture, IMU stabilization %s)",
            body_ctrl_.config().enabled ? "ON" : "OFF");
    }

private:
    void loadParameters()
    {
        // Gait
        GaitConfig gc = gait_.config();
        gc.period      = this->get_parameter("gait.period").as_double();
        gc.duty_factor = this->get_parameter("gait.duty_factor").as_double();
        gait_.setConfig(gc);

        // Trajectory
        TrajectoryConfig tc;
        tc.z_nominal   = this->get_parameter("trajectory.z_nominal").as_double();
        tc.step_height = this->get_parameter("trajectory.step_height").as_double();
        tc.step_amp_x  = this->get_parameter("trajectory.step_amp_x").as_double();
        tc.yaw_lever   = this->get_parameter("trajectory.yaw_lever").as_double();
        trajectory_.setConfig(tc);

        // Stabilization
        StabilizationConfig sc;
        sc.enabled          = this->get_parameter("stabilization.enabled").as_bool();
        sc.kp_roll          = this->get_parameter("stabilization.kp_roll").as_double();
        sc.kp_pitch         = this->get_parameter("stabilization.kp_pitch").as_double();
        sc.kd_roll          = this->get_parameter("stabilization.kd_roll").as_double();
        sc.kd_pitch         = this->get_parameter("stabilization.kd_pitch").as_double();
        sc.max_correction_z = this->get_parameter("stabilization.max_correction_z").as_double();
        sc.max_correction_x = this->get_parameter("stabilization.max_correction_x").as_double();
        body_ctrl_.setConfig(sc);

        // Filter
        filter_alpha_       = this->get_parameter("filter.alpha").as_double();
        stationary_thresh_  = this->get_parameter("filter.stationary_threshold").as_double();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_.vx   = msg->linear.x;
        cmd_vel_.vy   = msg->linear.y;
        cmd_vel_.vyaw = msg->angular.z;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto orient = BodyController::quaternionToEuler(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        body_ctrl_.updateOrientation(orient);

        // Периодическое логирование IMU (~2 сек)
        if (++imu_log_counter_ >= 100) {
            RCLCPP_DEBUG(this->get_logger(), "IMU: roll=%.3f pitch=%.3f",
                orient.roll, orient.pitch);
            imu_log_counter_ = 0;
        }
    }

    void timerCallback()
    {
        // Перезагрузка параметров (для runtime tuning)
        loadParameters();

        // Сглаживание скорости (low-pass filter)
        smoothed_vel_.vx   += (cmd_vel_.vx   - smoothed_vel_.vx)   * filter_alpha_;
        smoothed_vel_.vy   += (cmd_vel_.vy   - smoothed_vel_.vy)   * filter_alpha_;
        smoothed_vel_.vyaw += (cmd_vel_.vyaw - smoothed_vel_.vyaw) * filter_alpha_;

        QuadJoints joints;

        // Стойка если нет команды
        if (std::abs(smoothed_vel_.vx)   < stationary_thresh_ &&
            std::abs(smoothed_vel_.vy)   < stationary_thresh_ &&
            std::abs(smoothed_vel_.vyaw) < stationary_thresh_)
        {
            FootPosition stand = trajectory_.standingPose();
            LegJoints q = kinematics_.solveIK(stand, 1.0);
            for (int i = 0; i < LEG_COUNT; ++i) {
                joints[i] = q;
            }
            publishJoints(joints);
            return;
        }

        // Вычисляем фазы походки
        double elapsed = (this->now() - start_time_).seconds();
        auto phases = gait_.update(elapsed);

        // Коррекции от IMU
        auto corrections = body_ctrl_.computeCorrections();

        // Для каждой ноги: траектория + коррекция + IK
        for (int i = 0; i < LEG_COUNT; ++i) {
            FootPosition foot = trajectory_.compute(phases[i], smoothed_vel_, static_cast<LegId>(i));

            // Добавляем коррекцию стабилизации
            foot.x += corrections[i].x;
            foot.z += corrections[i].z;

            joints[i] = kinematics_.solveIK(foot, LEG_SIGN_Y[i]);
        }

        publishJoints(joints);
    }

    void publishJoints(const QuadJoints& joints)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(TOTAL_JOINTS);

        // Порядок: FL_hip, FL_thigh, FL_shin, FR_..., RL_..., RR_...
        for (int i = 0; i < LEG_COUNT; ++i) {
            msg.data[i * JOINTS_PER_LEG + 0] = joints[i].hip;
            msg.data[i * JOINTS_PER_LEG + 1] = joints[i].thigh;
            msg.data[i * JOINTS_PER_LEG + 2] = joints[i].knee;
        }

        publisher_->publish(msg);
    }

    // --- Библиотечные компоненты ---
    LegKinematics  kinematics_;
    GaitGenerator   gait_;
    FootTrajectory  trajectory_;
    BodyController  body_ctrl_;

    // --- ROS интерфейсы ---
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Time start_time_;

    // --- Состояние ---
    VelocityCommand cmd_vel_;
    VelocityCommand smoothed_vel_;

    // --- Параметры фильтра ---
    double filter_alpha_      = 0.1;
    double stationary_thresh_ = 0.01;

    // --- Счётчик логирования IMU ---
    int imu_log_counter_ = 0;
};

}  // namespace dog_brain

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dog_brain::TrotNode>());
    rclcpp::shutdown();
    return 0;
}
