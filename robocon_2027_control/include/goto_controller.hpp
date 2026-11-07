#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace robocon_control
{

inline double clamp(double v, double lo, double hi) {return std::max(lo, std::min(v, hi));}

inline double wrapAngle(double a) {return std::atan2(std::sin(a), std::cos(a));}

inline double yawFromQuaternion(double x, double y, double z, double w)
{
  return std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}

inline void globalToBody(double vx_g, double vy_g, double yaw, double & vx_b, double & vy_b)
{
  const double c = std::cos(yaw);
  const double s = std::sin(yaw);
  vx_b = c * vx_g + s * vy_g;
  vy_b = -s * vx_g + c * vy_g;
}

class PID
{
public:
  PID(double kp, double ki, double kd, double i_limit, double i_zone)
  : kp_(kp), ki_(ki), kd_(kd), i_limit_(i_limit), i_zone_(i_zone) {}

  void reset()
  {
    integral_ = 0.0;
    has_prev_ = false;
    prev_error_ = 0.0;
  }

  double update(double error, double dt)
  {
    if (dt <= 0.0) {
      return kp_ * error;
    }
    if (i_zone_ > 0.0 && std::fabs(error) > i_zone_) {
      integral_ = 0.0;  // jauh dari target: jangan menumpuk
    } else {
      integral_ += error * dt;
    }
    if (ki_ > 0.0) {
      const double bound = i_limit_ / ki_;
      integral_ = clamp(integral_, -bound, bound);
    }
    const double derivative = has_prev_ ? (error - prev_error_) / dt : 0.0;
    prev_error_ = error;
    has_prev_ = true;
    return kp_ * error + ki_ * integral_ + kd_ * derivative;
  }

private:
  double kp_, ki_, kd_, i_limit_, i_zone_;
  double integral_{0.0};
  double prev_error_{0.0};
  bool has_prev_{false};
};

struct Pose
{
  double x{0.0};
  double y{0.0};
  double yaw{0.0};
};

struct GoToParams
{
  double kp_pos{1.2}, ki_pos{0.05}, kd_pos{0.15}, i_limit_pos{0.15}, i_zone_pos{0.25};
  double kp_yaw{2.0}, ki_yaw{0.02}, kd_yaw{0.10}, i_limit_yaw{0.15}, i_zone_yaw{0.3};
  double max_linear_speed{0.6};
  double max_angular_speed{1.5};
  double tol_pos{0.03};
  double tol_yaw{0.05};
  double settle_time{0.3};
};

struct GoToConfig
{
  GoToParams ctrl;
  double linear_accel{1.5};
  double angular_accel{4.0};
  double timeout{60.0};  // detik; <= 0 berarti tanpa timeout
};

struct GoToOutput
{
  double vx{0.0}, vy{0.0}, wz{0.0};  // kerangka global
  bool reached{false};
  double dist{0.0};
  double yaw_error{0.0};
};

class GoToController
{
public:
  explicit GoToController(const GoToParams & p)
  : p_(p),
    pid_x_(p.kp_pos, p.ki_pos, p.kd_pos, p.i_limit_pos, p.i_zone_pos),
    pid_y_(p.kp_pos, p.ki_pos, p.kd_pos, p.i_limit_pos, p.i_zone_pos),
    pid_yaw_(p.kp_yaw, p.ki_yaw, p.kd_yaw, p.i_limit_yaw, p.i_zone_yaw) {}

  void setGoal(double x, double y, double yaw)
  {
    goal_ = Pose{x, y, wrapAngle(yaw)};
    pid_x_.reset();
    pid_y_.reset();
    pid_yaw_.reset();
    settled_ = 0.0;
  }

  const Pose & goal() const {return goal_;}

  GoToOutput update(const Pose & now, double dt)
  {
    GoToOutput o;
    const double ex = goal_.x - now.x;
    const double ey = goal_.y - now.y;
    const double eyaw = wrapAngle(goal_.yaw - now.yaw);
    o.dist = std::hypot(ex, ey);
    o.yaw_error = eyaw;

    o.vx = pid_x_.update(ex, dt);
    o.vy = pid_y_.update(ey, dt);
    o.wz = pid_yaw_.update(eyaw, dt);

    const double speed = std::hypot(o.vx, o.vy);
    if (speed > p_.max_linear_speed) {
      const double scale = p_.max_linear_speed / speed;
      o.vx *= scale;
      o.vy *= scale;
    }
    o.wz = clamp(o.wz, -p_.max_angular_speed, p_.max_angular_speed);

    const bool inside = o.dist < p_.tol_pos && std::fabs(eyaw) < p_.tol_yaw;
    settled_ = inside ? settled_ + dt : 0.0;
    o.reached = settled_ >= p_.settle_time;
    return o;
  }

private:
  GoToParams p_;
  PID pid_x_, pid_y_, pid_yaw_;
  Pose goal_;
  double settled_{0.0};
};

enum class GoToStatus {kIdle, kMoving, kReached, kTimeout, kCancelled};

inline std::string statusName(GoToStatus s)
{
  switch (s) {
    case GoToStatus::kIdle: return "IDLE";
    case GoToStatus::kMoving: return "MOVING";
    case GoToStatus::kReached: return "REACHED";
    case GoToStatus::kTimeout: return "TIMEOUT";
    case GoToStatus::kCancelled: return "CANCELLED";
  }
  return "IDLE";
}

struct StepResult
{
  bool send_cmd{false};   // terbitkan Twist (vx, vy, wz)
  bool finished{false};   // target selesai pada langkah ini (REACHED / TIMEOUT)
  GoToStatus status{GoToStatus::kIdle};
  double vx{0.0}, vy{0.0}, wz{0.0};  // kerangka badan robot
  double dist{0.0};
  double yaw_error{0.0};
};

/// Target yang sudah diterjemahkan ke frame odom.
struct ResolvedGoal
{
  double x{0.0}, y{0.0}, yaw{0.0};
  bool frame_known{true};
};

inline ResolvedGoal resolveGoal(
  const std::string & frame_in, double px, double py, bool has_yaw, double goal_yaw,
  const Pose & cur)
{
  std::string frame = frame_in;
  if (!frame.empty() && frame.front() == '/') {
    frame.erase(0, 1);
  }
  ResolvedGoal g;
  if (frame == "base_link" || frame == "base_footprint") {
    const double c = std::cos(cur.yaw);
    const double s = std::sin(cur.yaw);
    g.x = cur.x + c * px - s * py;
    g.y = cur.y + s * px + c * py;
    g.yaw = cur.yaw + (has_yaw ? goal_yaw : 0.0);
  } else {
    g.frame_known = frame.empty() || frame == "odom" || frame == "map";
    g.x = px;
    g.y = py;
    g.yaw = has_yaw ? goal_yaw : cur.yaw;
  }
  g.yaw = wrapAngle(g.yaw);
  return g;
}

class GoToRunner
{
public:
  explicit GoToRunner(const GoToConfig & cfg)
  : cfg_(cfg), controller_(cfg.ctrl) {}

  void setGoal(const Pose & goal)
  {
    controller_.setGoal(goal.x, goal.y, goal.yaw);
    active_ = true;
    elapsed_ = 0.0;
    status_ = GoToStatus::kMoving;
  }

  bool cancel()
  {
    if (!active_) {
      return false;
    }
    active_ = false;
    cmd_g_ = {0.0, 0.0, 0.0};
    status_ = GoToStatus::kCancelled;
    return true;
  }

  bool active() const {return active_;}
  GoToStatus status() const {return status_;}
  const Pose & goal() const {return controller_.goal();}

  StepResult step(const Pose & pose, double dt)
  {
    StepResult r;
    r.status = status_;
    if (!active_) {
      return r; 
    }
    elapsed_ += dt;

    const GoToOutput o = controller_.update(pose, dt);
    r.dist = o.dist;
    r.yaw_error = o.yaw_error;

    if (o.reached) {
      return finish(GoToStatus::kReached, r);
    }
    if (cfg_.timeout > 0.0 && elapsed_ > cfg_.timeout) {
      return finish(GoToStatus::kTimeout, r);
    }

    const double lin_step = cfg_.linear_accel * dt;
    const double ang_step = cfg_.angular_accel * dt;
    cmd_g_[0] = slew(cmd_g_[0], o.vx, lin_step);
    cmd_g_[1] = slew(cmd_g_[1], o.vy, lin_step);
    cmd_g_[2] = slew(cmd_g_[2], o.wz, ang_step);

    globalToBody(cmd_g_[0], cmd_g_[1], pose.yaw, r.vx, r.vy);
    r.wz = cmd_g_[2];
    r.send_cmd = true;
    return r;
  }

private:
  static double slew(double current, double target, double max_step)
  {
    const double delta = target - current;
    if (std::fabs(delta) <= max_step) {
      return target;
    }
    return current + std::copysign(max_step, delta);
  }

  StepResult finish(GoToStatus s, StepResult r)
  {
    active_ = false;
    cmd_g_ = {0.0, 0.0, 0.0};
    status_ = s;
    r.status = s;
    r.finished = true;
    return r;
  }

  GoToConfig cfg_;
  GoToController controller_;
  bool active_{false};
  double elapsed_{0.0};
  std::array<double, 3> cmd_g_{{0.0, 0.0, 0.0}};
  GoToStatus status_{GoToStatus::kIdle};
};

} 
