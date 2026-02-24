#include <chrono>
#include <memory>
#include <cmath>

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
        this->declare_parameter("gait.period", 0.8);
        this->declare_parameter("gait.duty_factor", 0.6);

        this->declare_parameter("trajectory.z_nominal", -0.25);
        this->declare_parameter("trajectory.x_standing", 0.0);
        this->declare_parameter("trajectory.step_height", 0.04);
        this->declare_parameter("trajectory.step_amp_x", 0.06);
        this->declare_parameter("trajectory.yaw_lever", 0.08);

        this->declare_parameter("stabilization.enabled", false);
        this->declare_parameter("stabilization.kp_roll", 0.3);
        this->declare_parameter("stabilization.kp_pitch", 0.3);
        this->declare_parameter("stabilization.kd_roll", 0.0);
        this->declare_parameter("stabilization.kd_pitch", 0.0);
        this->declare_parameter("stabilization.max_correction_z", 0.02);
        this->declare_parameter("stabilization.max_correction_x", 0.02);

        this->declare_parameter("filter.alpha", 0.1);
        this->declare_parameter("filter.stationary_threshold", 0.02);

        this->declare_parameter("startup.ramp_duration", 1.5);
        this->declare_parameter("startup.settle_time", 0.5);

        this->declare_parameter("cmd_vel_timeout", 0.5);

        loadParameters();

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
        last_cmd_time_ = this->now();

        // Compute standing joints once at startup
        FootPosition stand_pos = trajectory_.standingPose();
        standing_q_ = kinematics_.solveIK(stand_pos, 1.0);

        RCLCPP_INFO(this->get_logger(),
            "TrotNode: z_nominal=%.3f, standing thigh=%.3f knee=%.3f, ramp=%.1fs",
            trajectory_.config().z_nominal,
            standing_q_.thigh, standing_q_.knee,
            ramp_duration_);
    }

private:
    void loadParameters()
    {
        GaitConfig gc = gait_.config();
        gc.period      = this->get_parameter("gait.period").as_double();
        gc.duty_factor = this->get_parameter("gait.duty_factor").as_double();
        gait_.setConfig(gc);

        TrajectoryConfig tc;
        tc.z_nominal   = this->get_parameter("trajectory.z_nominal").as_double();
        tc.x_standing  = this->get_parameter("trajectory.x_standing").as_double();
        tc.step_height = this->get_parameter("trajectory.step_height").as_double();
        tc.step_amp_x  = this->get_parameter("trajectory.step_amp_x").as_double();
        tc.yaw_lever   = this->get_parameter("trajectory.yaw_lever").as_double();
        trajectory_.setConfig(tc);

        StabilizationConfig sc;
        sc.enabled          = this->get_parameter("stabilization.enabled").as_bool();
        sc.kp_roll          = this->get_parameter("stabilization.kp_roll").as_double();
        sc.kp_pitch         = this->get_parameter("stabilization.kp_pitch").as_double();
        sc.kd_roll          = this->get_parameter("stabilization.kd_roll").as_double();
        sc.kd_pitch         = this->get_parameter("stabilization.kd_pitch").as_double();
        sc.max_correction_z = this->get_parameter("stabilization.max_correction_z").as_double();
        sc.max_correction_x = this->get_parameter("stabilization.max_correction_x").as_double();
        body_ctrl_.setConfig(sc);

        filter_alpha_      = this->get_parameter("filter.alpha").as_double();
        stationary_thresh_ = this->get_parameter("filter.stationary_threshold").as_double();
        ramp_duration_     = this->get_parameter("startup.ramp_duration").as_double();
        settle_time_       = this->get_parameter("startup.settle_time").as_double();
        cmd_vel_timeout_   = this->get_parameter("cmd_vel_timeout").as_double();
    }

    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        cmd_vel_.vx   = msg->linear.x;
        cmd_vel_.vy   = msg->linear.y;
        cmd_vel_.vyaw = msg->angular.z;
        last_cmd_time_ = this->now();
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        auto orient = BodyController::quaternionToEuler(
            msg->orientation.x, msg->orientation.y,
            msg->orientation.z, msg->orientation.w);
        body_ctrl_.updateOrientation(orient);
    }

    void timerCallback()
    {
        double elapsed = (this->now() - start_time_).seconds();

        // === PHASE 1: Joint-space ramp from spawn to standing ===
        if (elapsed < ramp_duration_) {
            double t = elapsed / ramp_duration_;
            double alpha = t * t * (3.0 - 2.0 * t);  // smoothstep

            LegJoints q;
            q.hip   = 0.0;
            q.thigh = SPAWN_THIGH * (1.0 - alpha) + standing_q_.thigh * alpha;
            q.knee  = SPAWN_KNEE  * (1.0 - alpha) + standing_q_.knee  * alpha;

            QuadJoints joints;
            for (int i = 0; i < LEG_COUNT; ++i) joints[i] = q;
            publishJoints(joints);
            return;
        }

        // === PHASE 2: Hold standing (settle after ramp) ===
        if (elapsed < ramp_duration_ + settle_time_) {
            publishStanding();
            return;
        }

        // === PHASE 3: Normal operation ===

        // cmd_vel timeout: zero out if no message received recently.
        // teleop_twist_keyboard sends zero ONCE on key release — if that
        // packet is lost, cmd_vel_ stays nonzero forever without this.
        if ((this->now() - last_cmd_time_).seconds() > cmd_vel_timeout_) {
            cmd_vel_ = {};
        }

        // EWM filter on velocity
        smoothed_vel_.vx   += (cmd_vel_.vx   - smoothed_vel_.vx)   * filter_alpha_;
        smoothed_vel_.vy   += (cmd_vel_.vy   - smoothed_vel_.vy)   * filter_alpha_;
        smoothed_vel_.vyaw += (cmd_vel_.vyaw - smoothed_vel_.vyaw) * filter_alpha_;

        bool stationary = std::abs(smoothed_vel_.vx)   < stationary_thresh_ &&
                          std::abs(smoothed_vel_.vy)   < stationary_thresh_ &&
                          std::abs(smoothed_vel_.vyaw) < stationary_thresh_;

        if (stationary) {
            walking_ = false;
            publishStanding();
            return;
        }

        // Transition standing -> walking: reset gait clock so that
        // all legs start in double-support stance (no mid-swing jump).
        // gc=0.45 puts both diagonal pairs in stance simultaneously.
        if (!walking_) {
            walking_ = true;
            double period = gait_.config().period;
            gait_start_time_ = elapsed - 0.45 * period;
        }

        double walk_time = elapsed - gait_start_time_;
        auto phases = gait_.update(walk_time);
        auto corrections = body_ctrl_.computeCorrections();

        QuadJoints joints;
        for (int i = 0; i < LEG_COUNT; ++i) {
            FootPosition foot = trajectory_.compute(phases[i], smoothed_vel_, static_cast<LegId>(i));
            foot.x += corrections[i].x;
            foot.z += corrections[i].z;
            joints[i] = kinematics_.solveIK(foot, LEG_SIGN_Y[i]);
        }

        publishJoints(joints);
    }

    void publishStanding()
    {
        QuadJoints joints;
        for (int i = 0; i < LEG_COUNT; ++i) joints[i] = standing_q_;
        publishJoints(joints);
    }

    void publishJoints(const QuadJoints& joints)
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data.resize(TOTAL_JOINTS);
        for (int i = 0; i < LEG_COUNT; ++i) {
            msg.data[i * JOINTS_PER_LEG + 0] = joints[i].hip;
            msg.data[i * JOINTS_PER_LEG + 1] = joints[i].thigh;
            msg.data[i * JOINTS_PER_LEG + 2] = joints[i].knee;
        }
        publisher_->publish(msg);
    }

    // Must match URDF initial_value = physical spawn (shin upper limit = -0.1).
    static constexpr double SPAWN_THIGH = 0.0;
    static constexpr double SPAWN_KNEE  = -0.1;

    LegKinematics  kinematics_;
    GaitGenerator   gait_;
    FootTrajectory  trajectory_;
    BodyController  body_ctrl_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Time start_time_;
    rclcpp::Time last_cmd_time_;

    VelocityCommand cmd_vel_;
    VelocityCommand smoothed_vel_;
    LegJoints standing_q_;

    bool walking_ = false;
    double gait_start_time_ = 0.0;

    double filter_alpha_      = 0.1;
    double stationary_thresh_ = 0.02;
    double ramp_duration_     = 1.5;
    double settle_time_       = 0.5;
    double cmd_vel_timeout_   = 0.5;
};

}  // namespace dog_brain

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<dog_brain::TrotNode>());
    rclcpp::shutdown();
    return 0;
}
