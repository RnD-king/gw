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
        // =========================
        // Params
        // =========================
        this->declare_parameter<double>("cutoff_hz", 10.0);         // LPF cutoff (error에 적용)
        this->declare_parameter<double>("kp", 1.0);                 // P gain
        this->declare_parameter<double>("kd", 0.0);                 // D gain (측정값(now) 미분에 적용)
        this->declare_parameter<int>("max_tick_step_pitch", 200);   // pitch 루프 최대 tick 변화량
        this->declare_parameter<int>("max_tick_step_yaw", 200);     // yaw 루프 최대 tick 변화량
        this->declare_parameter<bool>("follow_slow_yaw", false);    // yaw HPF 사용 여부
        this->declare_parameter<double>("hpf_cutoff_hz", 0.3);      // yaw HPF 컷오프 주파수 (Hz)

        cutoff_hz_ = this->get_parameter("cutoff_hz").as_double();
        kp_        = this->get_parameter("kp").as_double();
        kd_        = this->get_parameter("kd").as_double();
        max_tick_step_pitch_ = this->get_parameter("max_tick_step_pitch").as_int();
        max_tick_step_yaw_   = this->get_parameter("max_tick_step_yaw").as_int();
        follow_slow_yaw_     = this->get_parameter("follow_slow_yaw").as_bool();
        hpf_cutoff_hz_       = this->get_parameter("hpf_cutoff_hz").as_double();

        if (max_tick_step_pitch_ < 1) max_tick_step_pitch_ = 1;
        if (max_tick_step_yaw_   < 1) max_tick_step_yaw_   = 1;

        yaw_lpf_ = 0.0; // HPF 상태 초기화

        // =========================
        // Dynamixel settings
        // =========================
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
        control_hz_ = 200.0;
        control_dt_ = 1.0 / control_hz_;

        prev_control_tp_ = steady_clock::now();
        last_dbg_tp_     = steady_clock::now();

        RCLCPP_INFO(this->get_logger(),
            "GimbalControlNode started. cutoff_hz=%.2f, kp=%.3f, kd=%.3f, max_step(P/Y)=%d/%d, control=%.1f Hz",
            cutoff_hz_, kp_, kd_, max_tick_step_pitch_, max_tick_step_yaw_, control_hz_);

        RCLCPP_INFO(this->get_logger(),
            "Waiting first IMU msg to calibrate (ref IMU only) ...");

        control_timer_ = this->create_wall_timer(
            duration<double>(control_dt_),
            std::bind(&GimbalControlNode::controlLoop, this)
        );
    }

private:
    // =========================
    // DXL helpers
    // =========================
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

    // =========================
    // Math helpers
    // =========================
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

    // =========================
    // IMU callback
    // =========================
    void imuCallback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mtx_);

        last_roll_  = msg->vector.y;
        last_pitch_ = msg->vector.x;
        last_yaw_   = msg->vector.z;

        imu_ready_ = true;

        if (!calibrated_) {
            ref_roll_  = last_roll_;
            ref_pitch_ = last_pitch_;
            ref_yaw_   = last_yaw_;

            err_pitch_lpf_ = 0.0;
            err_yaw_lpf_   = 0.0;
            prev_pitch_now_ = last_pitch_;
            prev_yaw_now_   = last_yaw_;

            calibrated_ = true;
            RCLCPP_INFO(this->get_logger(),
                "CALIB DONE: ref IMU(P/Y)=%.2f/%.2f deg",
                Rad2Deg(ref_pitch_), Rad2Deg(ref_yaw_));
        }
    }

    // =========================
    // Control loop
    // =========================
    void controlLoop()
    {
        double pitch_now, yaw_now, pitch_ref, yaw_ref;
        {
            std::lock_guard<std::mutex> lock(mtx_);
            if (!imu_ready_ || !calibrated_) return;
            pitch_now = last_pitch_;
            yaw_now   = last_yaw_;
            pitch_ref = ref_pitch_;
            yaw_ref   = ref_yaw_;
        }

        auto now_tp = steady_clock::now();
        double dt = duration<double>(now_tp - prev_control_tp_).count();
        prev_control_tp_ = now_tp;
        if (dt <= 0.0) return;
        double real_hz = 1.0 / dt;

        // 1) error
        double err_pitch = pitch_now - pitch_ref;
        double err_yaw_raw = WrapToPi(yaw_now - yaw_ref);
        double err_yaw_used;

        // HPF 적용
        if (follow_slow_yaw_) {
            double rc = 1.0 / (2.0 * M_PI * hpf_cutoff_hz_);
            double alpha = dt / (rc + dt);

            yaw_lpf_ += alpha * (yaw_now - yaw_lpf_);
            double yaw_hpf = err_yaw_raw - yaw_lpf_;
            err_yaw_used = yaw_hpf;
        } else {
            err_yaw_used = err_yaw_raw;
        }

        // 2) LPF on error (P항)
        if (cutoff_hz_ > 0.0) {
            double rc2 = 1.0 / (2.0 * M_PI * cutoff_hz_);
            double alpha2 = dt / (rc2 + dt);
            err_pitch_lpf_ += alpha2 * (err_pitch - err_pitch_lpf_);
            err_yaw_lpf_   += alpha2 * (err_yaw_used - err_yaw_lpf_);
        } else {
            err_pitch_lpf_ = err_pitch;
            err_yaw_lpf_   = err_yaw_used;
        }

        // 3) D term
        double pitch_rate = (pitch_now - prev_pitch_now_) / dt;
        double yaw_rate   = WrapToPi(yaw_now - prev_yaw_now_) / dt;
        prev_pitch_now_ = pitch_now;
        prev_yaw_now_   = yaw_now;

        // 4) PD command
        double cmd_pitch_rad = -(kp_ * err_pitch_lpf_ + kd_ * pitch_rate);
        double cmd_yaw_rad   = -(kp_ * err_yaw_lpf_   + kd_ * yaw_rate);

        // 5) delta ticks + clamp
        int delta_pitch_tick = radToTicks(cmd_pitch_rad);
        int delta_yaw_tick   = radToTicks(cmd_yaw_rad);

        delta_pitch_tick = std::clamp(delta_pitch_tick, -max_tick_step_pitch_, +max_tick_step_pitch_);
        delta_yaw_tick   = std::clamp(delta_yaw_tick,   -max_tick_step_yaw_,   +max_tick_step_yaw_);

        // 6) SyncRead present
        uint32_t p_present_u=0, y_present_u=0;
        bool ok_present = SyncReadPresent(p_present_u, y_present_u);
        int present_pitch = ok_present ? static_cast<int>(p_present_u) : static_cast<int>(readPosition(ID_PITCH_));
        int present_yaw   = ok_present ? static_cast<int>(y_present_u) : static_cast<int>(readPosition(ID_YAW_));

        // 7) goal = present + delta
        int goal_pitch_tick = std::clamp(present_pitch + delta_pitch_tick, 0, kTicksPerRev - 1);
        int goal_yaw_tick   = WrapTick(present_yaw + delta_yaw_tick);

        // 8) SyncWrite goal
        (void)SyncWriteGoal(goal_pitch_tick, goal_yaw_tick);

        // 9) Debug 1Hz
        auto now_dbg = steady_clock::now();
        if (duration<double>(now_dbg - last_dbg_tp_).count() >= 1.0) {
            last_dbg_tp_ = now_dbg;
            int yaw_goal_err_shortest = WrapDeltaTick(present_yaw, goal_yaw_tick);

            RCLCPP_INFO(
                this->get_logger(),
                "[DBG 1.0s] IMU now(P/Y)=%.2f/%.2f deg  ref(P/Y)=%.2f/%.2f deg  "
                "err_lpf(P/Y)=%.2f/%.2f deg  rate(P/Y)=%.2f/%.2f dps  "
                "cmd(P/Y)=%.2f/%.2f deg  delta_tick(P/Y)=%d/%d  "
                "present_tick(P/Y)=%d/%d  goal_tick(P/Y)=%d/%d  yaw_step_short=%d  real_hz=%.1f",
                Rad2Deg(pitch_now), Rad2Deg(yaw_now),
                Rad2Deg(pitch_ref), Rad2Deg(yaw_ref),
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
    double kp_{1.0};
    double kd_{0.0};
    int max_tick_step_pitch_{200};
    int max_tick_step_yaw_{200};

    bool follow_slow_yaw_{false};
    double hpf_cutoff_hz_{0.3};
    double yaw_lpf_{0.0};

    double control_hz_{200.0};
    double control_dt_{0.005};
    time_point<steady_clock> prev_control_tp_;
    time_point<steady_clock> last_dbg_tp_;

    std::mutex mtx_;
    bool imu_ready_{false};
    bool calibrated_{false};

    double last_roll_{0.0};
    double last_pitch_{0.0};
    double last_yaw_{0.0};

    double ref_roll_{0.0};
    double ref_pitch_{0.0};
    double ref_yaw_{0.0};   
    double err_pitch_lpf_{0.0};
    double err_yaw_lpf_{0.0};
    double prev_pitch_now_{0.0};
    double prev_yaw_now_{0.0};
};  

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}


