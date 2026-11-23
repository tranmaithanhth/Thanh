// Ackermann Terrain3D v1.0.3 - Gazebo Classic 11 + ROS 2 Humble
// Subscribes to std_msgs/Float64MultiArray [drive_force_N, steer_rad].
// In drive_mode=all, applies tau = F*R/4 to EACH of 4 wheels; drive_mode=rear uses F*R/2 at each rear.
// Ackermann PD actuates steering joints independently of traction motors.
// Unlike gazebo_ros_ackermann_drive, NEVER interprets force as target velocity.
#include <algorithm>
#include <cmath>
#include <functional>
#include <mutex>
#include <string>
#include <utility>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

namespace gazebo {
class AckermannForceDrive : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = std::move(model);
    node_ = gazebo_ros::Node::Get(sdf);
    if (!node_) { gzerr << "Failed to create ROS 2 gazebo node\n"; return; }
    auto joint = [&](const std::string &name) {
      auto found = model_->GetJoint(name);
      if (!found) RCLCPP_ERROR(node_->get_logger(), "Missing wheel/steer joint: %s", name.c_str());
      return found;
    };
    rear_l_ = joint("rear_left_wheel_joint");
    rear_r_ = joint("rear_right_wheel_joint");
    front_l_ = joint("front_left_wheel_joint");
    front_r_ = joint("front_right_wheel_joint");
    steer_l_ = joint("front_left_steer_joint");
    steer_r_ = joint("front_right_steer_joint");
    if (!(rear_l_ && rear_r_ && front_l_ && front_r_ && steer_l_ && steer_r_)) return;
    auto val = [&](const std::string &name, double default_value) {
      return sdf->Get<double>(name, default_value).first;
    };
    radius_ = val("wheel_radius", 0.15);
    wheelbase_ = val("wheelbase", 0.80);
    track_ = val("track_width", 0.50);
    max_force_ = val("max_force", 22.0 / 0.15);
    drive_mode_ = sdf->Get<std::string>("drive_mode", "rear").first;
    if (drive_mode_ != "rear" && drive_mode_ != "all") {
      RCLCPP_ERROR(node_->get_logger(), "Invalid drive_mode '%s': rear or all required", drive_mode_.c_str());
      return;
    }
    max_steer_ = val("max_steer", 0.52);
    max_steer_rate_ = val("max_steer_rate", 0.95);
    kp_ = val("steering_kp", 25.0);
    kd_ = val("steering_kd", 3.0);
    max_steer_torque_ = val("max_steer_torque", 25.0);
    timeout_s_ = val("command_timeout", 0.6);
    if (radius_ <= 0 || wheelbase_ <= 0 || track_ <= 0 || max_force_ <= 0 || timeout_s_ <= 0) {
      RCLCPP_ERROR(node_->get_logger(), "Invalid Ackermann force plugin parameters"); return;
    }
    // Message convention is deliberately explicit. Do not publish Twist cmd_vel here.
    cmd_sub_ = node_->create_subscription<std_msgs::msg::Float64MultiArray>(
      "force_steer", rclcpp::QoS(10),
      [this](const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if (msg->data.size() < 2 || !std::isfinite(msg->data[0]) || !std::isfinite(msg->data[1])) return;
        std::lock_guard<std::mutex> lock(mutex_);
        target_force_ = std::clamp(msg->data[0], -max_force_, max_force_);
        target_delta_ = std::clamp(msg->data[1], -max_steer_, max_steer_);
        last_cmd_sim_ = model_->GetWorld()->SimTime();
        seen_cmd_ = true;
      });
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("odom", rclcpp::QoS(10));
    state_pub_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>("force_state", rclcpp::QoS(10));
    last_update_ = model_->GetWorld()->SimTime();
    last_publish_ = last_update_;
    update_connection_ = event::Events::ConnectWorldUpdateBegin(
      std::bind(&AckermannForceDrive::OnUpdate, this, std::placeholders::_1));
    RCLCPP_INFO(node_->get_logger(),
      "Force Ackermann ready: |F|<=%.2f N, |delta|<=%.3f rad, drive=%s",
      max_force_, max_steer_, drive_mode_.c_str());
  }

private:
  void OnUpdate(const common::UpdateInfo &info) {
    // Gazebo ODE expects SetForce to be issued on EVERY simulation step.
    // The physics step is 0.001 s in this world. Do not throttle actuation
    // to 100 Hz: doing so produces only a small fraction of requested torque.
    const double dt = (info.simTime - last_update_).Double();
    if (dt <= 0.0) return;
    const double capped_dt = std::min(dt, 0.05);
    last_update_ = info.simTime;
    double commanded_f = 0, commanded_delta = 0;
    {
      std::lock_guard<std::mutex> lock(mutex_);
      if (seen_cmd_ && (info.simTime - last_cmd_sim_).Double() <= timeout_s_) {
        commanded_f = target_force_;
        commanded_delta = target_delta_;
      }
    }
    // Enforce steering actuation rate, independent of ROS command frequency.
    const double increment = std::clamp(commanded_delta - reference_delta_,
                                        -max_steer_rate_ * capped_dt, max_steer_rate_ * capped_dt);
    reference_delta_ = std::clamp(reference_delta_ + increment, -max_steer_, max_steer_);
    const double tangent = std::tan(reference_delta_);
    const double delta_l = std::atan2(tangent, 1.0 - 0.5 * track_ / wheelbase_ * tangent);
    const double delta_r = std::atan2(tangent, 1.0 + 0.5 * track_ / wheelbase_ * tangent);
    const auto pd = [&](const physics::JointPtr &j, double desired) {
      const double torque = kp_ * (desired - j->Position(0)) - kd_ * j->GetVelocity(0);
      j->SetForce(0, std::clamp(torque, -max_steer_torque_, max_steer_torque_));
    };
    pd(steer_l_, delta_l);
    pd(steer_r_, delta_r);
    // cmd F is a torque-equivalent TOTAL drive force, not guaranteed contact traction.
    // In all-wheel-drive mode the same total torque is split four ways to increase
    // available tire contact traction on grades. Rear-drive mode keeps MATLAB's
    // rear-axle-only configuration for A/B comparisons.
    const double total_torque = std::clamp(commanded_f, -max_force_, max_force_) * radius_;
    const double each_rear_torque = total_torque / (drive_mode_ == "all" ? 4.0 : 2.0);
    const double each_front_torque = drive_mode_ == "all" ? total_torque / 4.0 : 0.0;
    rear_l_->SetForce(0, each_rear_torque);
    rear_r_->SetForce(0, each_rear_torque);
    front_l_->SetForce(0, each_front_torque);
    front_r_->SetForce(0, each_front_torque);

    // The actuator loop is every physics step, but ROS publications are only
    // 50 Hz. Keep this gate BELOW all wheel and steering SetForce calls.
    if ((info.simTime - last_publish_).Double() < 0.02) return;
    last_publish_ = info.simTime;

    const auto pose = model_->WorldPose();
    const auto lv = model_->WorldLinearVel();
    const auto av = model_->WorldAngularVel();
    const double yaw = pose.Rot().Yaw();
    nav_msgs::msg::Odometry odom;
    odom.header.frame_id = "world";
    odom.header.stamp.sec = static_cast<int32_t>(info.simTime.sec);
    odom.header.stamp.nanosec = static_cast<uint32_t>(info.simTime.nsec);
    odom.child_frame_id = "base_footprint";
    odom.pose.pose.position.x = pose.Pos().X();
    odom.pose.pose.position.y = pose.Pos().Y();
    odom.pose.pose.position.z = pose.Pos().Z();
    odom.pose.pose.orientation.x = pose.Rot().X();
    odom.pose.pose.orientation.y = pose.Rot().Y();
    odom.pose.pose.orientation.z = pose.Rot().Z();
    odom.pose.pose.orientation.w = pose.Rot().W();
    odom.twist.twist.linear.x = std::cos(yaw) * lv.X() + std::sin(yaw) * lv.Y();
    odom.twist.twist.linear.y = -std::sin(yaw) * lv.X() + std::cos(yaw) * lv.Y();
    odom.twist.twist.linear.z = lv.Z();
    odom.twist.twist.angular.z = av.Z();
    odom_pub_->publish(odom);
    std_msgs::msg::Float64MultiArray state;
    // First 6 elements are backwards-compatible. Extra: front wheel rad/s,
    // applied rear/front torques per wheel (Nm). All values are signed.
    state.data = {commanded_f, reference_delta_, (steer_l_->Position(0) + steer_r_->Position(0)) * 0.5,
                  rear_l_->GetVelocity(0), rear_r_->GetVelocity(0), odom.twist.twist.linear.x,
                  front_l_->GetVelocity(0), front_r_->GetVelocity(0),
                  each_rear_torque, each_front_torque};
    state_pub_->publish(state);
  }

  physics::ModelPtr model_;
  physics::JointPtr rear_l_, rear_r_, front_l_, front_r_, steer_l_, steer_r_;
  gazebo_ros::Node::SharedPtr node_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr state_pub_;
  event::ConnectionPtr update_connection_;
  std::mutex mutex_;
  common::Time last_cmd_sim_;
  common::Time last_update_;
  common::Time last_publish_;
  bool seen_cmd_ = false;
  double target_force_ = 0.0, target_delta_ = 0.0, reference_delta_ = 0.0;
  double radius_ = 0.15, wheelbase_ = 0.8, track_ = 0.5;
  std::string drive_mode_ = "rear";
  double max_force_ = 146.6667, max_steer_ = 0.52, max_steer_rate_ = 0.95;
  double kp_ = 25, kd_ = 3, max_steer_torque_ = 25, timeout_s_ = 0.6;
};
GZ_REGISTER_MODEL_PLUGIN(AckermannForceDrive)
} // namespace gazebo