#include <chrono>
#include <cmath>
#include <atomic>
#include <algorithm>
#include <string>
#include <thread>
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono;

class FrequencyTest : public rclcpp::Node
{
public:
    FrequencyTest() : Node("frequency_test")
    {
        port_name_ = "/dev/ttyUSB0";
        baud_ = 4000000;
        id_ = 21;

        ADDR_TORQUE_ENABLE    = 64;
        ADDR_GOAL_POSITION    = 116;
        ADDR_PRESENT_POSITION = 132;

        freq_ = this->declare_parameter<double>("freq", 100.0);
        dt_   = 1.0 / freq_;

        kp_      = this->declare_parameter<double>("kp", 0.3);
        u_limit_ = this->declare_parameter<int>("u_limit", 300);

        tx_settle_us_ = this->declare_parameter<int>("tx_settle_us", 200);

        portHandler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!portHandler_->openPort())
            RCLCPP_FATAL(get_logger(), "Failed to open port: %s", port_name_.c_str());

        if (!portHandler_->setBaudRate(baud_))
            RCLCPP_FATAL(get_logger(), "Failed to set baud rate: %d", baud_);

        // Torque enable (확인용 TxRx)
        {
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, id_, ADDR_TORQUE_ENABLE, 1, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS)
                RCLCPP_FATAL(get_logger(), "Torque enable comm error: %s",
                             packetHandler_->getTxRxResult(dxl_comm_result));
            if (dxl_error != 0)
                RCLCPP_FATAL(get_logger(), "Torque enable dxl_error: %u", dxl_error);
        }

        prev_time_ = steady_clock::now();
        step_ = 0;

        last_ref_.store(0);
        last_now_.store(0);
        last_cmd_.store(0);
        last_real_hz_.store(0.0);

        stat_count_.store(0);
        stat_dt_sum_.store(0.0);
        stat_dt_min_.store(1e9);
        stat_dt_max_.store(0.0);

        read_ok_count_.store(0);
        read_fail_count_.store(0);

        last_fail_code_.store(COMM_SUCCESS);
        last_dxl_error_.store(0);
        last_fail_msg_.store(0); // 0=none, 1=timeout, 2=corrupt, 3=other, 4=dxl_error

        loop_timer_ = create_wall_timer(
            duration<double>(dt_),
            std::bind(&FrequencyTest::controlLoop, this)
        );

        log_timer_ = create_wall_timer(
            1s,
            std::bind(&FrequencyTest::logLoop, this)
        );
    }

private:
    void controlLoop() // 제어주파수마다
    {
        auto now_tp = steady_clock::now();
        double dt_real = duration<double>(now_tp - prev_time_).count();
        prev_time_ = now_tp;

        if (dt_real > 0.0) {
            last_real_hz_.store(1.0 / dt_real);

            stat_count_.fetch_add(1);
            stat_dt_sum_.store(stat_dt_sum_.load() + dt_real);
            if (dt_real < stat_dt_min_.load()) stat_dt_min_.store(dt_real);
            if (dt_real > stat_dt_max_.load()) stat_dt_max_.store(dt_real);
        }

        // 1. sin 함수로 동작
        int ref = 2048 + static_cast<int>(200.0 * std::sin(step_ * 0.1));
        last_ref_.store(ref);

        // 2. 엔코더 읽기 < TxRx
        uint8_t dxl_error = 0;
        uint32_t now_pos_u32 = 0;
        int dxl_comm_result = packetHandler_->read4ByteTxRx(
            portHandler_, id_, ADDR_PRESENT_POSITION, &now_pos_u32, &dxl_error);

        if (dxl_comm_result != COMM_SUCCESS || dxl_error != 0) { // 만약 통신 실패하면 바로 리턴 < 딜레이 최소화
            read_fail_count_.fetch_add(1);

            last_fail_code_.store(dxl_comm_result);
            last_dxl_error_.store(dxl_error);

            if (dxl_error != 0) {
                last_fail_msg_.store(4); // 에러 메세지 
            } else {
                const char* msg = packetHandler_->getTxRxResult(dxl_comm_result);
                if (msg && std::string(msg).find("Timeout") != std::string::npos) {
                    last_fail_msg_.store(1);
                } else if (msg && std::string(msg).find("Corrupt") != std::string::npos) {
                    last_fail_msg_.store(2);
                } else {
                    last_fail_msg_.store(3);
                }
            }

            step_++;
            return; // 실패 시 write 안 함
        }

        read_ok_count_.fetch_add(1); // 성공
        last_fail_msg_.store(0);
        last_fail_code_.store(COMM_SUCCESS);
        last_dxl_error_.store(0);

        int now_pos = static_cast<int>(now_pos_u32);
        last_now_.store(now_pos);

        // 3. 간단한 P제어
        int err = ref - now_pos;
        int u = static_cast<int>(kp_ * static_cast<double>(err));
        u = std::clamp(u, -u_limit_, u_limit_);

        int cmd = ref + u;
        cmd = std::clamp(cmd, 0, 4095);
        last_cmd_.store(cmd);

        // 4. 명령 쓰기 < TxOnly
        packetHandler_->write4ByteTxOnly(portHandler_, id_, ADDR_GOAL_POSITION, cmd);

        // 5. TxOnly 직후 아주 짧게 대기해서 버스/USB 버퍼 정리 시간 확보
        if (tx_settle_us_ > 0) {
            std::this_thread::sleep_for(std::chrono::microseconds(tx_settle_us_));
        }

        step_++;
    }

    static const char* FailMsgToStr(int r) // 실패 원인 출력 < 없으면 0
    {
        switch (r) {
            case 0: return "none";
            case 1: return "timeout";
            case 2: return "corrupt";
            case 3: return "comm_other";
            case 4: return "dxl_error";
            default: return "unknown";
        }
    }

    void logLoop() // 1초마다
    {
        int n = stat_count_.exchange(0);
        double sum = stat_dt_sum_.exchange(0.0);
        double mn = stat_dt_min_.exchange(1e9);
        double mx = stat_dt_max_.exchange(0.0);

        int ok = read_ok_count_.exchange(0);
        int fail = read_fail_count_.exchange(0);

        int fail_msg = last_fail_msg_.load();
        int fail_code = last_fail_code_.load();
        int dxl_err = last_dxl_error_.load();

        double mean_dt = (n > 0) ? (sum / static_cast<double>(n)) : 0.0;
        double mean_hz = (mean_dt > 0.0) ? (1.0 / mean_dt) : 0.0;

        int ref = last_ref_.load();
        int now = last_now_.load();
        int cmd = last_cmd_.load();
        int diff = std::abs(ref - now);

        const char* comm_str = packetHandler_->getTxRxResult(fail_code);

        RCLCPP_INFO(
            get_logger(),
            "[req=%.1fHz | mean=%.1fHz | inst=%.1fHz | dt(ms) mean=%.2f min=%.2f max=%.2f | read ok=%d fail=%d | last_fail=%s(code=%d, %s) dxl_error=%d | kp=%.3f | tx_settle_us=%d] ref=%d now=%d cmd=%d diff=%d",
            freq_,
            mean_hz,
            last_real_hz_.load(),
            mean_dt * 1000.0,
            (mn < 1e8 ? mn * 1000.0 : 0.0),
            mx * 1000.0,
            ok, fail,
            FailMsgToStr(fail_msg),
            fail_code,
            (comm_str ? comm_str : "null"),
            dxl_err,
            kp_,
            tx_settle_us_,
            ref, now, cmd, diff
        );
    }

    // Dynamixel
    dynamixel::PortHandler  *portHandler_;
    dynamixel::PacketHandler *packetHandler_;

    // 설정
    std::string port_name_;
    int baud_;
    int id_;

    int ADDR_TORQUE_ENABLE;
    int ADDR_GOAL_POSITION;
    int ADDR_PRESENT_POSITION;

    double freq_;
    double dt_;

    double kp_;
    int u_limit_;
    int tx_settle_us_;

    // ROS 타이머
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::TimerBase::SharedPtr log_timer_;

    // 상태
    size_t step_;
    time_point<steady_clock> prev_time_;

    std::atomic<int>    last_ref_;
    std::atomic<int>    last_now_;
    std::atomic<int>    last_cmd_;
    std::atomic<double> last_real_hz_;

    // 1초 통계
    std::atomic<int>    stat_count_;
    std::atomic<double> stat_dt_sum_;
    std::atomic<double> stat_dt_min_;
    std::atomic<double> stat_dt_max_;

    std::atomic<int> read_ok_count_;
    std::atomic<int> read_fail_count_;

    std::atomic<int> last_fail_code_;
    std::atomic<int> last_dxl_error_;
    std::atomic<int> last_fail_msg_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrequencyTest>());
    rclcpp::shutdown();
    return 0;
}
