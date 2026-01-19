#include <chrono>
#include <cmath>
#include <mutex>
#include <algorithm>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono;
using std::placeholders::_1;

class GimbalControlNode : public rclcpp::Node
{
public:
  GimbalControlNode()
  : Node("gimbal_control")
  {
    this->declare_parameter<double>("cutoff_hz", 15.0);  
    this->declare_parameter<double>("kp_pitch", 1.0);       
    this->declare_parameter<double>("kd_pitch", 0.01);  
    this->declare_parameter<double>("ki_pitch", 0.0);
    this->declare_parameter<double>("kp_yaw", 1.8);               
    this->declare_parameter<double>("kd_yaw", 0.02);                 
    this->declare_parameter<double>("ki_yaw", 0.0);
    this->declare_parameter<int>("max_tick_step_pitch", 200);   // pitch 루프 최대 tick 변화량
    this->declare_parameter<int>("max_tick_step_yaw", 200);   // yaw 루프 최대 tick 변화량

    cutoff_hz_ = this->get_parameter("cutoff_hz").as_double();
    kp_pitch_ = this->get_parameter("kp_pitch").as_double();
    kd_pitch_ = this->get_parameter("kd_pitch").as_double();
    ki_pitch_ = this->get_parameter("ki_pitch").as_double();
    kp_yaw_ = this->get_parameter("kp_yaw").as_double();
    kd_yaw_ = this->get_parameter("kd_yaw").as_double();
    ki_yaw_ = this->get_parameter("ki_yaw").as_double();

    max_tick_step_pitch_ = this->get_parameter("max_tick_step_pitch").as_int();
    max_tick_step_yaw_   = this->get_parameter("max_tick_step_yaw").as_int();
    if (max_tick_step_pitch_ < 1) max_tick_step_pitch_ = 1;
    if (max_tick_step_yaw_   < 1) max_tick_step_yaw_   = 1;

    port_name_ = "/dev/ttyUSB0";
    baud_      = 4000000;

    ID_YAW_   = 21;
    ID_PITCH_ = 22;

    ADDR_TORQUE_ENABLE_    = 64;
    ADDR_GOAL_POSITION_    = 116;
    ADDR_PRESENT_POSITION_ = 132;

    LEN_GOAL_POSITION_    = 4;
    LEN_PRESENT_POSITION_ = 4;

    portHandler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler_->openPort()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open port %s", port_name_.c_str());
    }
    if (!portHandler_->setBaudRate(baud_)) {
      RCLCPP_ERROR(this->get_logger(), "Failed to set baudrate %d", baud_);
    }

    // SyncRead / SyncWrite
    groupSyncRead_ = std::make_unique<dynamixel::GroupSyncRead>(
      portHandler_, packetHandler_, ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);

    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(
      portHandler_, packetHandler_, ADDR_GOAL_POSITION_, LEN_GOAL_POSITION_);

    if (!groupSyncRead_->addParam(ID_PITCH_)) {
      RCLCPP_ERROR(this->get_logger(), "GroupSyncRead addParam failed (PITCH)");
    }
    if (!groupSyncRead_->addParam(ID_YAW_)) {
      RCLCPP_ERROR(this->get_logger(), "GroupSyncRead addParam failed (YAW)");
    }

    uint8_t dxl_error = 0;
    packetHandler_->write1ByteTxRx(portHandler_, ID_YAW_,   ADDR_TORQUE_ENABLE_, 1, &dxl_error);
    packetHandler_->write1ByteTxRx(portHandler_, ID_PITCH_, ADDR_TORQUE_ENABLE_, 1, &dxl_error);

    uint32_t p0=0, y0=0;
    if (SyncReadPresent(p0, y0)) {
      base_pitch_tick_ = p0;
      base_yaw_tick_   = y0;
    } else {
      base_pitch_tick_ = readPosition(ID_PITCH_);
      base_yaw_tick_   = readPosition(ID_YAW_);
    }

    RCLCPP_INFO(this->get_logger(), "Base Pitch tick = %u", base_pitch_tick_);
    RCLCPP_INFO(this->get_logger(), "Base Yaw   tick = %u", base_yaw_tick_);

    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/camera/imu_tilt", 50,
      std::bind(&GimbalControlNode::imuCallback, this, _1)
    );

    control_hz_ = 200.0;
    control_dt_ = 1.0 / control_hz_;

    prev_control_tp_ = steady_clock::now();
    last_dbg_tp_     = steady_clock::now();

    RCLCPP_INFO(this->get_logger(),
      "GimbalControlNode started. cutoff_hz=%.2f, max_step(P/Y)=%d/%d, control=%.1f Hz",
      cutoff_hz_, max_tick_step_pitch_, max_tick_step_yaw_, control_hz_);

    RCLCPP_INFO(this->get_logger(),
      "Waiting first IMU msg ...");

    control_timer_ = this->create_wall_timer(
      duration<double>(control_dt_),
      std::bind(&GimbalControlNode::controlLoop, this)
    );
  }

private:
  uint32_t readPosition(int id)
  {
    uint8_t dxl_error = 0;
    uint32_t pos = 0;
    int rc = packetHandler_->read4ByteTxRx(portHandler_, id, ADDR_PRESENT_POSITION_, &pos, &dxl_error);
    if (rc != COMM_SUCCESS) {
      RCLCPP_WARN(this->get_logger(), "DXL read error (id=%d): %s", id, packetHandler_->getTxRxResult(rc));
    }
    return pos;
  }

  // SyncRead / SyncWrite
  bool SyncReadPresent(uint32_t &pitch_pos, uint32_t &yaw_pos)
  {
    int rc = groupSyncRead_->txRxPacket();
    if (rc != COMM_SUCCESS) return false;

    bool ok_p = groupSyncRead_->isAvailable(ID_PITCH_, ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);
    bool ok_y = groupSyncRead_->isAvailable(ID_YAW_,   ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);
    if (!ok_p || !ok_y) return false;

    pitch_pos = groupSyncRead_->getData(ID_PITCH_, ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);
    yaw_pos   = groupSyncRead_->getData(ID_YAW_,   ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);
    return true;
  }

  bool SyncWriteGoal(int goal_pitch_tick, int goal_yaw_tick)
  {
    goal_pitch_tick = WrapTick(goal_pitch_tick);
    goal_yaw_tick   = WrapTick(goal_yaw_tick);

    uint8_t p_param[4] = {
      DXL_LOBYTE(DXL_LOWORD(goal_pitch_tick)),
      DXL_HIBYTE(DXL_LOWORD(goal_pitch_tick)),
      DXL_LOBYTE(DXL_HIWORD(goal_pitch_tick)),
      DXL_HIBYTE(DXL_HIWORD(goal_pitch_tick))
    };
    uint8_t y_param[4] = {
      DXL_LOBYTE(DXL_LOWORD(goal_yaw_tick)),
      DXL_HIBYTE(DXL_LOWORD(goal_yaw_tick)),
      DXL_LOBYTE(DXL_HIWORD(goal_yaw_tick)),
      DXL_HIBYTE(DXL_HIWORD(goal_yaw_tick))
    };

    groupSyncWrite_->clearParam();

    bool ok1 = groupSyncWrite_->addParam(ID_PITCH_, p_param);
    bool ok2 = groupSyncWrite_->addParam(ID_YAW_,   y_param);
    if (!ok1 || !ok2) {
      groupSyncWrite_->clearParam();
      return false;
    }

    int rc = groupSyncWrite_->txPacket();
    groupSyncWrite_->clearParam();

    return (rc == COMM_SUCCESS);
  }

  static constexpr int kTicksPerRev = 4096;

  int radToTicks(double rad) const
  {
    double tick_d = rad / (2.0 * M_PI) * static_cast<double>(kTicksPerRev);
    return static_cast<int>(std::round(tick_d));
  }

  static double Rad2Deg(double r) { return r * 180.0 / M_PI; }

  static double WrapToPi(double a)
  {
    while (a >  M_PI) a -= 2.0 * M_PI;
    while (a < -M_PI) a += 2.0 * M_PI;
    return a;
  }

  // tick을 -2048, 2047 범위로 래핑
  static int WrapTick(int t)
  {
    int m = t % kTicksPerRev;
    if (m < 0) m += kTicksPerRev;
    return m;
  }

  static int WrapDeltaTick(int from, int to)
  {
    int d = WrapTick(to) - WrapTick(from);
    if (d >  kTicksPerRev/2) d -= kTicksPerRev;
    if (d < -kTicksPerRev/2) d += kTicksPerRev;
    return d;
  }

  void imuCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    last_roll_  = msg->vector.y;
    last_pitch_ = msg->vector.x;
    last_yaw_   = msg->vector.z;

    if (!imu_ready_) {
      // Initialize history for rate/LPF on first IMU message.
      prev_pitch_now_ = last_pitch_;
      prev_yaw_now_   = last_yaw_;
      err_pitch_lpf_  = 0.0;
      err_yaw_lpf_    = 0.0;
    }

    imu_ready_ = true;
  }

  void controlLoop()
  {
    double pitch_now, yaw_now;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!imu_ready_) return;

      pitch_now = last_pitch_;
      yaw_now   = last_yaw_;
    }

    auto now_tp = steady_clock::now();
    double dt = duration<double>(now_tp - prev_control_tp_).count();
    prev_control_tp_ = now_tp;
    if (dt <= 0.0) return;
    double real_hz = 1.0 / dt;

    // 1) error = now - ref
    double err_pitch = pitch_now;
    double err_yaw   = WrapToPi(yaw_now);

    // 2) LPF 적용
    if (cutoff_hz_ > 0.0) {
      double rc = 1.0 / (2.0 * M_PI * cutoff_hz_);
      double a  = dt / (rc + dt);
      err_pitch_lpf_ += a * (err_pitch - err_pitch_lpf_);
      err_yaw_lpf_   += a * (err_yaw   - err_yaw_lpf_);
    } else {
      err_pitch_lpf_ = err_pitch;
      err_yaw_lpf_   = err_yaw;
    }

    double pitch_rate = (pitch_now - prev_pitch_now_) / dt;
    double yaw_rate   = WrapToPi(yaw_now - prev_yaw_now_) / dt; // yaw는 랩핑 후 미분
    prev_pitch_now_ = pitch_now;
    prev_yaw_now_   = yaw_now;

    // 3) PID
    err_pitch_integral_ += err_pitch_lpf_ * dt;
    err_yaw_integral_   += err_yaw_lpf_   * dt;

    double max_pitch_integral = radToTicks(max_tick_step_pitch_) / (ki_pitch_ > 0.0 ? ki_pitch_ : 1.0);
    double max_yaw_integral   = radToTicks(max_tick_step_yaw_)   / (ki_yaw_ > 0.0 ? ki_yaw_ : 1.0);

    err_pitch_integral_ = std::clamp(err_pitch_integral_, -max_pitch_integral, +max_pitch_integral);
    err_yaw_integral_   = std::clamp(err_yaw_integral_,   -max_yaw_integral,   +max_yaw_integral);

    double cmd_pitch_rad = -(kp_pitch_ * err_pitch_lpf_ + kd_pitch_ * pitch_rate + ki_pitch_ * err_pitch_integral_);
    double cmd_yaw_rad   = -(kp_yaw_ * err_yaw_lpf_   + kd_yaw_ * yaw_rate   + ki_yaw_ * err_yaw_integral_);

    //  4) delta ticks
    int delta_pitch_tick = radToTicks(cmd_pitch_rad);
    int delta_yaw_tick   = radToTicks(cmd_yaw_rad);

    delta_pitch_tick = std::clamp(delta_pitch_tick, -max_tick_step_pitch_, +max_tick_step_pitch_);
    delta_yaw_tick   = std::clamp(delta_yaw_tick,   -max_tick_step_yaw_,   +max_tick_step_yaw_);

    uint32_t p_present_u=0, y_present_u=0;
    bool ok_present = SyncReadPresent(p_present_u, y_present_u);
    int present_pitch = ok_present ? static_cast<int>(p_present_u) : static_cast<int>(readPosition(ID_PITCH_));
    int present_yaw   = ok_present ? static_cast<int>(y_present_u) : static_cast<int>(readPosition(ID_YAW_));

    // 5) goal = present + delta
    int goal_pitch_tick = std::clamp(present_pitch + delta_pitch_tick, 0, kTicksPerRev - 1);
    int goal_yaw_tick   = WrapTick(present_yaw + delta_yaw_tick);

    (void)SyncWriteGoal(goal_pitch_tick, goal_yaw_tick);

    auto now_dbg = steady_clock::now();
    if (duration<double>(now_dbg - last_dbg_tp_).count() >= 1.0) { // 1초마다 
      last_dbg_tp_ = now_dbg;

      int yaw_goal_err_shortest = WrapDeltaTick(present_yaw, goal_yaw_tick);

      RCLCPP_INFO(
        this->get_logger(),
        "[DBG 1.0s] IMU now(P/Y)=%.2f/%.2f deg  "
        "err_lpf(P/Y)=%.2f/%.2f deg  rate(P/Y)=%.2f/%.2f dps  "
        "cmd(P/Y)=%.2f/%.2f deg  delta_tick(P/Y)=%d/%d  "
        "present_tick(P/Y)=%d/%d  goal_tick(P/Y)=%d/%d  yaw_step_short=%d  real_hz=%.1f",
        Rad2Deg(pitch_now), Rad2Deg(yaw_now),
        Rad2Deg(err_pitch_lpf_), Rad2Deg(err_yaw_lpf_),
        Rad2Deg(pitch_rate), Rad2Deg(yaw_rate),
        Rad2Deg(cmd_pitch_rad), Rad2Deg(cmd_yaw_rad),
        delta_pitch_tick, delta_yaw_tick,
        present_pitch, present_yaw,
        goal_pitch_tick, goal_yaw_tick,
        yaw_goal_err_shortest,
        real_hz
      );
    }
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  dynamixel::PortHandler *portHandler_{nullptr};
  dynamixel::PacketHandler *packetHandler_{nullptr};

  std::unique_ptr<dynamixel::GroupSyncRead>  groupSyncRead_;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;

  std::string port_name_;
  int baud_{4000000};

  int ID_YAW_{21}, ID_PITCH_{22};
  int ADDR_TORQUE_ENABLE_{64};
  int ADDR_GOAL_POSITION_{116};
  int ADDR_PRESENT_POSITION_{132};
  int LEN_GOAL_POSITION_{4};
  int LEN_PRESENT_POSITION_{4};

  uint32_t base_yaw_tick_{0};
  uint32_t base_pitch_tick_{0};

  double cutoff_hz_{10.0};
  double kp_pitch_{1.0};
  double kd_pitch_{0.0};
  double ki_pitch_{0.0};
  double kp_yaw_{1.0};
  double kd_yaw_{0.0};
  double ki_yaw_{0.0};

  int    max_tick_step_pitch_{200};
  int    max_tick_step_yaw_{200};

  double control_hz_{200.0};
  double control_dt_{0.005};
  time_point<steady_clock> prev_control_tp_;
  time_point<steady_clock> last_dbg_tp_;

  std::mutex mtx_;
  bool imu_ready_{false};
  double last_roll_{0.0};
  double last_pitch_{0.0};
  double last_yaw_{0.0};

  double err_pitch_lpf_{0.0};
  double err_yaw_lpf_{0.0};

  double prev_pitch_now_{0.0};
  double prev_yaw_now_{0.0};

  double err_pitch_integral_{0.0};
  double err_yaw_integral_{0.0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalControlNode>());
  rclcpp::shutdown();
  return 0;
}
