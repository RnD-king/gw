#include <chrono>
#include <cmath>
#include <mutex>
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
  : Node("gimbal_control_direct")
  {
    // =========================
    // Dynamixel settings
    // =========================
    port_name_ = "/dev/ttyUSB0";
    baud_      = 4000000;

    ID_YAW_   = 21;
    ID_PITCH_ = 22;

    ADDR_TORQUE_ENABLE_     = 64;
    ADDR_GOAL_POSITION_     = 116;
    ADDR_PRESENT_POSITION_  = 132;

    LEN_GOAL_POSITION_     = 4;
    LEN_PRESENT_POSITION_  = 4;

    portHandler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
    packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

    if (!portHandler_->openPort()) {
      RCLCPP_FATAL(this->get_logger(), "Failed to open port");
    }
    if (!portHandler_->setBaudRate(baud_)) {
      RCLCPP_FATAL(this->get_logger(), "Failed to set baudrate");
    }

    groupSyncRead_ = std::make_unique<dynamixel::GroupSyncRead>(
      portHandler_, packetHandler_,
      ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);

    groupSyncWrite_ = std::make_unique<dynamixel::GroupSyncWrite>(
      portHandler_, packetHandler_,
      ADDR_GOAL_POSITION_, LEN_GOAL_POSITION_);

    groupSyncRead_->addParam(ID_PITCH_);
    groupSyncRead_->addParam(ID_YAW_);

    uint8_t dxl_error = 0;
    packetHandler_->write1ByteTxRx(portHandler_, ID_YAW_,   ADDR_TORQUE_ENABLE_, 1, &dxl_error);
    packetHandler_->write1ByteTxRx(portHandler_, ID_PITCH_, ADDR_TORQUE_ENABLE_, 1, &dxl_error);

    // =========================
    // Read base tick
    // =========================
    SyncReadPresent(base_pitch_tick_, base_yaw_tick_);

    RCLCPP_INFO(this->get_logger(),
      "Base tick Pitch=%u Yaw=%u",
      base_pitch_tick_, base_yaw_tick_);

    // =========================
    // IMU subscription
    // =========================
    imu_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
      "/camera/imu_tilt", 50,
      std::bind(&GimbalControlNode::imuCallback, this, _1)
    );

    // =========================
    // Control timer
    // =========================
    control_timer_ = this->create_wall_timer(
      duration<double>(0.005),
      std::bind(&GimbalControlNode::controlLoop, this)
    );

    RCLCPP_INFO(this->get_logger(), "Gimbal DIRECT tracking started");
  }

private:
  // =========================
  // Constants / helpers
  // =========================
  static constexpr int kTicksPerRev = 4096;

  static int WrapTick(int t)
  {
    int m = t % kTicksPerRev;
    if (m < 0) m += kTicksPerRev;
    return m;
  }

  static int radToTicks(double rad)
  {
    return static_cast<int>(
      std::round(rad / (2.0 * M_PI) * kTicksPerRev)
    );
  }

  // =========================
  // Sync helpers
  // =========================
  bool SyncReadPresent(uint32_t &pitch, uint32_t &yaw)
  {
    if (groupSyncRead_->txRxPacket() != COMM_SUCCESS)
      return false;

    pitch = groupSyncRead_->getData(ID_PITCH_, ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);
    yaw   = groupSyncRead_->getData(ID_YAW_,   ADDR_PRESENT_POSITION_, LEN_PRESENT_POSITION_);
    return true;
  }

  bool SyncWriteGoal(int pitch, int yaw)
  {
    pitch = std::clamp(pitch, 0, kTicksPerRev - 1);
    yaw   = WrapTick(yaw);

    uint8_t p_param[4] = {
      DXL_LOBYTE(DXL_LOWORD(pitch)),
      DXL_HIBYTE(DXL_LOWORD(pitch)),
      DXL_LOBYTE(DXL_HIWORD(pitch)),
      DXL_HIBYTE(DXL_HIWORD(pitch))
    };

    uint8_t y_param[4] = {
      DXL_LOBYTE(DXL_LOWORD(yaw)),
      DXL_HIBYTE(DXL_LOWORD(yaw)),
      DXL_LOBYTE(DXL_HIWORD(yaw)),
      DXL_HIBYTE(DXL_HIWORD(yaw))
    };

    groupSyncWrite_->clearParam();
    groupSyncWrite_->addParam(ID_PITCH_, p_param);
    groupSyncWrite_->addParam(ID_YAW_,   y_param);

    bool ok = (groupSyncWrite_->txPacket() == COMM_SUCCESS);
    groupSyncWrite_->clearParam();
    return ok;
  }

  // =========================
  // IMU callback
  // =========================
  void imuCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mtx_);

    imu_pitch_ = msg->vector.x;
    imu_yaw_   = msg->vector.z;

    if (!calibrated_) {
      ref_pitch_ = imu_pitch_;
      ref_yaw_   = imu_yaw_;
      calibrated_ = true;

      RCLCPP_INFO(this->get_logger(),
        "IMU reference set (P/Y)=%.2f / %.2f deg",
        imu_pitch_ * 180.0 / M_PI,
        imu_yaw_   * 180.0 / M_PI);
    }
  }

  // =========================
  // Control loop
  // =========================
  void controlLoop()
  {
    if (!calibrated_) return;

    double pitch, yaw;
    {
      std::lock_guard<std::mutex> lock(mtx_);
      pitch = imu_pitch_ - ref_pitch_;
      yaw   = imu_yaw_   - ref_yaw_;
    }

    int pitch_tick = base_pitch_tick_ - radToTicks(pitch);
    int yaw_tick   = base_yaw_tick_   - radToTicks(yaw);

    SyncWriteGoal(pitch_tick, yaw_tick);
  }

private:
  // ===== ROS =====
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr imu_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  // ===== DXL =====
  dynamixel::PortHandler *portHandler_{nullptr};
  dynamixel::PacketHandler *packetHandler_{nullptr};
  std::unique_ptr<dynamixel::GroupSyncRead>  groupSyncRead_;
  std::unique_ptr<dynamixel::GroupSyncWrite> groupSyncWrite_;

  std::string port_name_;
  int baud_;

  int ID_YAW_, ID_PITCH_;
  int ADDR_TORQUE_ENABLE_;
  int ADDR_GOAL_POSITION_;
  int ADDR_PRESENT_POSITION_;
  int LEN_GOAL_POSITION_;
  int LEN_PRESENT_POSITION_;

  // ===== State =====
  std::mutex mtx_;
  bool calibrated_{false};

  double imu_pitch_{0.0};
  double imu_yaw_{0.0};
  double ref_pitch_{0.0};
  double ref_yaw_{0.0};

  uint32_t base_pitch_tick_{0};
  uint32_t base_yaw_tick_{0};
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GimbalControlNode>());
  rclcpp::shutdown();
  return 0;
}
