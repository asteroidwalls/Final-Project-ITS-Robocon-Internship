#include <chrono>
#include <cmath>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/string.hpp"

#include "goto_controller.hpp"

using namespace robocon_control; 

class TRGoToPID : public rclcpp::Node
{
  using Clock = std::chrono::steady_clock;

public:
  TRGoToPID()
  : Node("tr_goto_pid"), runner_(loadConfig())
  {
    const auto goal_topic = declare_parameter<std::string>("goal_topic", "/move_base_simple/goal");
    const auto cancel_topic = declare_parameter<std::string>("cancel_topic", "/goto/cancel");
    const auto status_topic = declare_parameter<std::string>("status_topic", "/goto/status");
    const auto odom_topic = declare_parameter<std::string>("odom_topic", "/odom");
    const auto cmd_vel_topic = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel");

    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, 10,
      [this](nav_msgs::msg::Odometry::ConstSharedPtr msg) {odomCallback(*msg);});
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      goal_topic, 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {goalCallback(*msg);});
    cancel_sub_ = create_subscription<std_msgs::msg::Empty>(
      cancel_topic, 10,
      [this](std_msgs::msg::Empty::ConstSharedPtr) {cancelCallback();});

    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / rate_)),
      [this]() {onTimer();});

    publishStatus("IDLE");
    RCLCPP_INFO(get_logger(), "tr_goto_pid siap. Kirim target ke %s (PoseStamped).", goal_topic.c_str());
  }

  void stopIfActive()
  {
    if (runner_.cancel()) {
      sendZero();
      publishStatus("CANCELLED");
    }
  }

private:
  GoToConfig loadConfig()
  {
    GoToConfig c;
    rate_ = declare_parameter<double>("control_rate", 30.0);
    c.ctrl.kp_pos = declare_parameter<double>("kp_pos", c.ctrl.kp_pos);
    c.ctrl.ki_pos = declare_parameter<double>("ki_pos", c.ctrl.ki_pos);
    c.ctrl.kd_pos = declare_parameter<double>("kd_pos", c.ctrl.kd_pos);
    c.ctrl.i_limit_pos = declare_parameter<double>("i_limit_pos", c.ctrl.i_limit_pos);
    c.ctrl.i_zone_pos = declare_parameter<double>("i_zone_pos", c.ctrl.i_zone_pos);
    c.ctrl.kp_yaw = declare_parameter<double>("kp_yaw", c.ctrl.kp_yaw);
    c.ctrl.ki_yaw = declare_parameter<double>("ki_yaw", c.ctrl.ki_yaw);
    c.ctrl.kd_yaw = declare_parameter<double>("kd_yaw", c.ctrl.kd_yaw);
    c.ctrl.i_limit_yaw = declare_parameter<double>("i_limit_yaw", c.ctrl.i_limit_yaw);
    c.ctrl.i_zone_yaw = declare_parameter<double>("i_zone_yaw", c.ctrl.i_zone_yaw);
    c.ctrl.max_linear_speed = declare_parameter<double>("max_linear_speed", c.ctrl.max_linear_speed);
    c.ctrl.max_angular_speed = declare_parameter<double>("max_angular_speed", c.ctrl.max_angular_speed);
    c.ctrl.tol_pos = declare_parameter<double>("tol_pos", c.ctrl.tol_pos);
    c.ctrl.tol_yaw = declare_parameter<double>("tol_yaw", c.ctrl.tol_yaw);
    c.ctrl.settle_time = declare_parameter<double>("settle_time", c.ctrl.settle_time);
    c.linear_accel = declare_parameter<double>("linear_accel", c.linear_accel);
    c.angular_accel = declare_parameter<double>("angular_accel", c.angular_accel);
    c.timeout = declare_parameter<double>("timeout", c.timeout);
    return c;
  }

  void odomCallback(const nav_msgs::msg::Odometry & msg)
  {
    const auto & q = msg.pose.pose.orientation;
    pose_.x = msg.pose.pose.position.x;
    pose_.y = msg.pose.pose.position.y;
    pose_.yaw = yawFromQuaternion(q.x, q.y, q.z, q.w);
    odom_received_ = true;
  }

  void goalCallback(const geometry_msgs::msg::PoseStamped & msg)
  {
    if (!odom_received_) {
      RCLCPP_WARN(get_logger(), "Target diabaikan: /odom belum diterima.");
      return;
    }
    const auto & pos = msg.pose.position;
    const auto & q = msg.pose.orientation;
    const double norm = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    const bool has_yaw = norm >= 1e-6;
    const double goal_yaw = has_yaw ? yawFromQuaternion(q.x / norm, q.y / norm, q.z / norm, q.w / norm) : 0.0;

    const ResolvedGoal g = resolveGoal(msg.header.frame_id, pos.x, pos.y, has_yaw, goal_yaw, pose_);
    if (!g.frame_known) {
      RCLCPP_WARN(
        get_logger(), "Frame '%s' tidak dikenal, target dianggap di frame odom.",
        msg.header.frame_id.c_str());
    }

    runner_.setGoal(Pose{g.x, g.y, g.yaw});
    have_last_tick_ = false;
    RCLCPP_INFO(get_logger(), "Target baru: x=%.3f y=%.3f yaw=%.1f deg", g.x, g.y, g.yaw * 180.0 / M_PI);
    publishStatus("MOVING");
  }

  void cancelCallback()
  {
    if (runner_.cancel()) {
      sendZero();
      RCLCPP_INFO(get_logger(), "Selesai: CANCELLED");
      publishStatus("CANCELLED");
    }
  }

  void onTimer()
  {
    const auto now = Clock::now();
    double dt = 1.0 / rate_;
    if (have_last_tick_) {
      dt = std::chrono::duration<double>(now - last_tick_).count();
    }
    last_tick_ = now;
    have_last_tick_ = true;
    step(clamp(dt, 0.001, 0.2));
  }

  void step(double dt)
  {
    if (!runner_.active()) {
      return;  
    }
    const StepResult r = runner_.step(pose_, dt);

    if (r.finished) {
      sendZero();
      const std::string name = statusName(r.status);
      RCLCPP_INFO(get_logger(), "Selesai: %s", name.c_str());
      publishStatus(name);
      return;
    }
    if (r.send_cmd) {
      geometry_msgs::msg::Twist cmd;
      cmd.linear.x = r.vx;
      cmd.linear.y = r.vy;
      cmd.angular.z = r.wz;
      cmd_pub_->publish(cmd);

      if (std::chrono::duration<double>(Clock::now() - last_status_).count() > 0.5) {
        std::ostringstream os;
        os << "MOVING jarak=" << std::fixed << std::setprecision(3) << r.dist
           << " m galat_yaw=" << std::setprecision(1) << r.yaw_error * 180.0 / M_PI << " deg";
        publishStatus(os.str());
      }
    }
  }

  void sendZero()
  {
    for (int i = 0; i < 3; ++i) {
      cmd_pub_->publish(geometry_msgs::msg::Twist());
    }
  }

  void publishStatus(const std::string & text)
  {
    last_status_ = Clock::now();
    std_msgs::msg::String msg;
    msg.data = text;
    status_pub_->publish(msg);
  }

  double rate_{30.0};
  GoToRunner runner_;
  Pose pose_;
  bool odom_received_{false};
  bool have_last_tick_{false};
  Clock::time_point last_tick_{};
  Clock::time_point last_status_{};

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr cancel_sub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<TRGoToPID>();
  rclcpp::spin(node);
  try {
    node->stopIfActive();  
  } catch (...) {
  }
  rclcpp::shutdown();
  return 0;
}
