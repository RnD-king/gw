#include <chrono>
#include <cmath>
#include <atomic>
#include <algorithm>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono;

class FrequencyTestMulti : public rclcpp::Node
{
public:
    FrequencyTestMulti() : Node("frequency_test_multi")
    {
        port_name_ = "/dev/ttyUSB0";
        baud_ = 4000000;

        id_first_ = this->declare_parameter<int>("id_first", 0);
        id_last_  = this->declare_parameter<int>("id_last", 22);

        ADDR_TORQUE_ENABLE    = 64;
        ADDR_GOAL_POSITION    = 116;
        ADDR_PRESENT_POSITION = 132;

        LEN_GOAL_POSITION     = 4;
        LEN_PRESENT_POSITION  = 4;

        // 주파수
        freq_ = this->declare_parameter<double>("freq", 1000.0);
        dt_   = 1.0 / freq_;

        // sin 움직임 파라미터
        amp_      = this->declare_parameter<int>("amp", 20);
        sin_hz_   = this->declare_parameter<double>("sin_hz", 0.5);
        phase_step_rad_ = this->declare_parameter<double>("phase_step_rad", 0.15);

        // 안전 tick 범위 제한
        min_tick_ = this->declare_parameter<int>("min_tick", 0);
        max_tick_ = this->declare_parameter<int>("max_tick", 4095);

        tx_settle_us_ = this->declare_parameter<int>("tx_settle_us", 0);

        kp_      = this->declare_parameter<double>("kp", 0.3);
        u_limit_ = this->declare_parameter<int>("u_limit", 300);

        // 모터 ID 리스트
        for (int id = id_first_; id <= id_last_; ++id) ids_.push_back(id);
        motor_n_ = static_cast<int>(ids_.size());

        // ---------------- center_ticks 입력 ----------------
        // [중요] center_ticks_는 "ids_와 1:1 대응" (index = i)
        const std::vector<int> CENTER_TICKS = {
            /*i0  (ID0) */  2048,
            /*i1  (ID1) */  2048,
            /*i2  (ID2) */  2674,
            /*i3  (ID3) */  1422,
            /*i4  (ID4) */  1834,
            /*i5  (ID5) */  2262,
            /*i6  (ID6) */  1536,
            /*i7  (ID7) */  2560,
            /*i8  (ID8) */  2048,
            /*i9  (ID9) */  2048,
            /*i10 (ID10)*/  2048,
            /*i11 (ID11)*/  2048,
            /*i12 (ID12)*/  2048,
            /*i13 (ID13)*/  3072,
            /*i14 (ID14)*/  1024,
            /*i15 (ID15)*/  1365,
            /*i16 (ID16)*/  2731,
            /*i17 (ID17)*/  1024,
            /*i18 (ID18)*/  3072,
            /*i19 (ID19)*/  2048,
            /*i20 (ID20)*/  2048,
            /*i21 (ID21)*/  2048,
            /*i22 (ID22)*/  1800
        };

        center_ticks_ = CENTER_TICKS;

        if (static_cast<int>(center_ticks_.size()) != motor_n_) {
            RCLCPP_FATAL(
                get_logger(),
                "center_ticks size mismatch: expected %d (IDs %d..%d) but got %zu. Exiting.",
                motor_n_, id_first_, id_last_, center_ticks_.size()
            );
            rclcpp::shutdown();
            return;
        }

        for (int i = 0; i < motor_n_; ++i) {
            if (center_ticks_[i] < min_tick_ || center_ticks_[i] > max_tick_) {
                RCLCPP_FATAL(
                    get_logger(),
                    "center_ticks[%d] (= %d) out of range [%d, %d]. Exiting for safety.",
                    i, center_ticks_[i], min_tick_, max_tick_
                );
                rclcpp::shutdown();
                return;
            }
        }
        // ---------------------------------------------------

        last_ref_.assign(motor_n_, 0);
        last_now_.assign(motor_n_, 0);
        last_cmd_.assign(motor_n_, 0);

        valid_now_.assign(motor_n_, false);

        portHandler_   = dynamixel::PortHandler::getPortHandler(port_name_.c_str());
        packetHandler_ = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!portHandler_->openPort())
            RCLCPP_FATAL(get_logger(), "Failed to open port: %s", port_name_.c_str());

        if (!portHandler_->setBaudRate(baud_))
            RCLCPP_FATAL(get_logger(), "Failed to set baud rate: %d", baud_);

        sync_write_ = new dynamixel::GroupSyncWrite(
            portHandler_, packetHandler_, ADDR_GOAL_POSITION, LEN_GOAL_POSITION);

        sync_read_  = new dynamixel::GroupSyncRead(
            portHandler_, packetHandler_, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);

        for (int id : ids_) {
            if (!sync_read_->addParam(id)) {
                RCLCPP_FATAL(get_logger(), "GroupSyncRead addParam failed for ID=%d", id);
            }
        }

        // Torque ON
        for (int id : ids_) {
            uint8_t dxl_error = 0;
            int dxl_comm_result = packetHandler_->write1ByteTxRx(
                portHandler_, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);

            if (dxl_comm_result != COMM_SUCCESS) {
                RCLCPP_FATAL(get_logger(), "Torque enable comm error ID=%d: %s",
                             id, packetHandler_->getTxRxResult(dxl_comm_result));
            }
            if (dxl_error != 0) {
                RCLCPP_FATAL(get_logger(), "Torque enable dxl_error ID=%d: %u", id, dxl_error);
            }
        }

        // 통계 초기화
        prev_time_ = steady_clock::now();
        t_sec_ = 0.0;

        last_real_hz_.store(0.0);

        stat_count_.store(0);
        stat_dt_sum_.store(0.0);
        stat_dt_min_.store(1e9);
        stat_dt_max_.store(0.0);

        read_ok_count_.store(0);
        read_fail_count_.store(0);
        write_ok_count_.store(0);
        write_fail_count_.store(0);
        avail_miss_count_.store(0);

        last_fail_code_.store(COMM_SUCCESS);
        last_fail_msg_.store(0);

        // 프로파일링 통계 초기화
        prof_loop_dt_.Reset();
        prof_sr_txrx_.Reset();
        prof_sr_parse_.Reset();
        prof_build_cmd_.Reset();
        prof_sw_tx_.Reset();
        prof_settle_.Reset();
        prof_total_.Reset();

        loop_timer_ = create_wall_timer(
            duration<double>(dt_),
            std::bind(&FrequencyTestMulti::controlLoop, this)
        );

        log_timer_ = create_wall_timer(
            1s,
            std::bind(&FrequencyTestMulti::logLoop, this)
        );

        RCLCPP_INFO(get_logger(),
            "Start: IDs [%d..%d] N=%d, req=%.1fHz, sin_hz=%.3f, amp=%d, tick_range=[%d,%d]",
            id_first_, id_last_, motor_n_, freq_, sin_hz_, amp_, min_tick_, max_tick_);
    }

    ~FrequencyTestMulti() override
    {
        if (sync_write_) { delete sync_write_; sync_write_ = nullptr; }
        if (sync_read_)  { delete sync_read_;  sync_read_  = nullptr; }
        if (portHandler_) portHandler_->closePort();
    }

private:
    // -------------------- Profiling helper --------------------
    static void AtomicUpdateMin(std::atomic<double>& target, double v)
    {
        double cur = target.load(std::memory_order_relaxed);
        while (v < cur && !target.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
            // cur updated by compare_exchange_weak
        }
    }

    static void AtomicUpdateMax(std::atomic<double>& target, double v)
    {
        double cur = target.load(std::memory_order_relaxed);
        while (v > cur && !target.compare_exchange_weak(cur, v, std::memory_order_relaxed)) {
            // cur updated
        }
    }

    struct StageStat
    {
        std::atomic<int> count{0};
        std::atomic<double> sum_s{0.0};
        std::atomic<double> min_s{1e9};
        std::atomic<double> max_s{0.0};

        void Reset()
        {
            count.store(0, std::memory_order_relaxed);
            sum_s.store(0.0, std::memory_order_relaxed);
            min_s.store(1e9, std::memory_order_relaxed);
            max_s.store(0.0, std::memory_order_relaxed);
        }

        void Add(double seconds)
        {
            count.fetch_add(1, std::memory_order_relaxed);
            sum_s.store(sum_s.load(std::memory_order_relaxed) + seconds, std::memory_order_relaxed);
            AtomicUpdateMin(min_s, seconds);
            AtomicUpdateMax(max_s, seconds);
        }

        void Consume(int &n, double &sum, double &mn, double &mx)
        {
            n = count.exchange(0, std::memory_order_relaxed);
            sum = sum_s.exchange(0.0, std::memory_order_relaxed);
            mn = min_s.exchange(1e9, std::memory_order_relaxed);
            mx = max_s.exchange(0.0, std::memory_order_relaxed);
        }
    };
    // ----------------------------------------------------------

    void controlLoop()
    {
        const auto tp_loop_begin = steady_clock::now();

        // ---- loop_dt 측정 (콜백 간격) ----
        auto now_tp = tp_loop_begin;
        double dt_real = duration<double>(now_tp - prev_time_).count();
        prev_time_ = now_tp;

        if (dt_real > 0.0) {
            last_real_hz_.store(1.0 / dt_real);

            stat_count_.fetch_add(1);
            stat_dt_sum_.store(stat_dt_sum_.load() + dt_real);
            if (dt_real < stat_dt_min_.load()) stat_dt_min_.store(dt_real);
            if (dt_real > stat_dt_max_.load()) stat_dt_max_.store(dt_real);

            t_sec_ += dt_real;

            // 프로파일: loop_dt
            prof_loop_dt_.Add(dt_real);
        }

        // ============================================================
        // 1) SyncRead txRxPacket 시간
        // ============================================================
        const auto tp_sr_begin = steady_clock::now();
        int dxl_comm_result = sync_read_->txRxPacket();
        const auto tp_sr_end = steady_clock::now();
        prof_sr_txrx_.Add(duration<double>(tp_sr_end - tp_sr_begin).count());

        if (dxl_comm_result != COMM_SUCCESS) {
            read_fail_count_.fetch_add(1);
            last_fail_code_.store(dxl_comm_result);
            last_fail_msg_.store(ClassifyFailMsg(dxl_comm_result));

            // total time
            prof_total_.Add(duration<double>(steady_clock::now() - tp_loop_begin).count());
            return;
        }
        read_ok_count_.fetch_add(1);

        // ============================================================
        // 2) SyncRead parse 시간 (isAvailable/getData)
        // ============================================================
        const auto tp_parse_begin = steady_clock::now();
        std::fill(valid_now_.begin(), valid_now_.end(), false);

        for (int i = 0; i < motor_n_; ++i) {
            int id = ids_[i];
            if (!sync_read_->isAvailable(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)) {
                avail_miss_count_.fetch_add(1);
                continue;
            }
            uint32_t now_u32 = sync_read_->getData(id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION);
            last_now_[i] = static_cast<int>(now_u32);
            valid_now_[i] = true;
        }
        const auto tp_parse_end = steady_clock::now();
        prof_sr_parse_.Add(duration<double>(tp_parse_end - tp_parse_begin).count());

        // ============================================================
        // 3) ref/P/param build 시간
        // ============================================================
        const auto tp_build_begin = steady_clock::now();

        const double w = 2.0 * M_PI * sin_hz_;
        sync_write_->clearParam();

        int added_count = 0;
        for (int i = 0; i < motor_n_; ++i) {
            int id = ids_[i];
            int center = center_ticks_[i];

            double phase = phase_step_rad_ * static_cast<double>(i);
            int ref = center + static_cast<int>(std::round(
                static_cast<double>(amp_) * std::sin(w * t_sec_ + phase)
            ));
            ref = std::clamp(ref, min_tick_, max_tick_);
            last_ref_[i] = ref;

            if (!valid_now_[i]) {
                last_cmd_[i] = center;
                continue;
            }

            int now_pos = last_now_[i];

            int err = ref - now_pos;
            int u = static_cast<int>(std::round(kp_ * static_cast<double>(err)));
            u = std::clamp(u, -u_limit_, u_limit_);

            int cmd = ref + u;
            cmd = std::clamp(cmd, min_tick_, max_tick_);
            last_cmd_[i] = cmd;

            uint8_t param_goal[4];
            param_goal[0] = DXL_LOBYTE(DXL_LOWORD(cmd));
            param_goal[1] = DXL_HIBYTE(DXL_LOWORD(cmd));
            param_goal[2] = DXL_LOBYTE(DXL_HIWORD(cmd));
            param_goal[3] = DXL_HIBYTE(DXL_HIWORD(cmd));

            if (!sync_write_->addParam(id, param_goal)) {
                write_fail_count_.fetch_add(1);
            } else {
                added_count++;
            }
        }

        const auto tp_build_end = steady_clock::now();
        prof_build_cmd_.Add(duration<double>(tp_build_end - tp_build_begin).count());

        // write할 모터가 하나도 없으면 txPacket 생략
        if (added_count == 0) {
            last_fail_msg_.store(0);
            last_fail_code_.store(COMM_SUCCESS);

            prof_total_.Add(duration<double>(steady_clock::now() - tp_loop_begin).count());
            return;
        }

        // ============================================================
        // 4) SyncWrite txPacket 시간
        // ============================================================
        const auto tp_sw_begin = steady_clock::now();
        dxl_comm_result = sync_write_->txPacket();
        const auto tp_sw_end = steady_clock::now();
        prof_sw_tx_.Add(duration<double>(tp_sw_end - tp_sw_begin).count());

        if (dxl_comm_result != COMM_SUCCESS) {
            write_fail_count_.fetch_add(1);
            last_fail_code_.store(dxl_comm_result);
            last_fail_msg_.store(ClassifyFailMsg(dxl_comm_result));

            prof_total_.Add(duration<double>(steady_clock::now() - tp_loop_begin).count());
            return;
        }
        write_ok_count_.fetch_add(1);

        // ============================================================
        // 5) settle sleep 시간
        // ============================================================
        if (tx_settle_us_ > 0) {
            const auto tp_settle_begin = steady_clock::now();
            std::this_thread::sleep_for(std::chrono::microseconds(tx_settle_us_));
            const auto tp_settle_end = steady_clock::now();
            prof_settle_.Add(duration<double>(tp_settle_end - tp_settle_begin).count());
        } else {
            // 의미는 없지만, count를 맞추고 싶으면 Add(0)도 가능
            // prof_settle_.Add(0.0);
        }

        last_fail_msg_.store(0);
        last_fail_code_.store(COMM_SUCCESS);

        // total time
        prof_total_.Add(duration<double>(steady_clock::now() - tp_loop_begin).count());
    }

    static int ClassifyFailMsg(int comm_code)
    {
        if (comm_code == COMM_RX_TIMEOUT) return 1;
        if (comm_code == COMM_RX_CORRUPT) return 2;
        return 3;
    }

    static const char* FailMsgToStr(int r)
    {
        switch (r) {
            case 0: return "none";
            case 1: return "timeout";
            case 2: return "corrupt";
            case 3: return "comm_other";
            default: return "unknown";
        }
    }

    static void StatToMs(int n, double sum_s, double mn_s, double mx_s,
                         double &mean_ms, double &mn_ms, double &mx_ms)
    {
        if (n <= 0) {
            mean_ms = mn_ms = mx_ms = 0.0;
            return;
        }
        mean_ms = (sum_s / static_cast<double>(n)) * 1000.0;
        mn_ms = (mn_s < 1e8 ? mn_s * 1000.0 : 0.0);
        mx_ms = mx_s * 1000.0;
    }

    void logLoop()
    {
        // ---------- 기존 주파수/통신 통계 ----------
        int n = stat_count_.exchange(0);
        double sum = stat_dt_sum_.exchange(0.0);
        double mn = stat_dt_min_.exchange(1e9);
        double mx = stat_dt_max_.exchange(0.0);

        int rok = read_ok_count_.exchange(0);
        int rfail = read_fail_count_.exchange(0);
        int wok = write_ok_count_.exchange(0);
        int wfail = write_fail_count_.exchange(0);

        int avail_miss = avail_miss_count_.exchange(0);

        int fail_msg = last_fail_msg_.load();
        int fail_code = last_fail_code_.load();

        double mean_dt = (n > 0) ? (sum / static_cast<double>(n)) : 0.0;
        double mean_hz = (mean_dt > 0.0) ? (1.0 / mean_dt) : 0.0;

        int idx0 = 0;
        int idxN = motor_n_ - 1;

        const char* comm_str = packetHandler_->getTxRxResult(fail_code);

        RCLCPP_INFO(
            get_logger(),
            "[IDs %d..%d | req=%.1fHz | mean=%.1fHz | inst=%.1fHz | dt(ms) mean=%.3f min=%.3f max=%.3f | "
            "SyncWrite ok=%d fail=%d | SyncRead ok=%d fail=%d | avail_miss=%d | last_fail=%s(code=%d, %s)] "
            "ID%d(center=%d cmd=%d now=%d v=%d)  ID%d(center=%d cmd=%d now=%d v=%d)",
            id_first_, id_last_,
            freq_,
            mean_hz,
            last_real_hz_.load(),
            mean_dt * 1000.0,
            (mn < 1e8 ? mn * 1000.0 : 0.0),
            mx * 1000.0,
            wok, wfail,
            rok, rfail,
            avail_miss,
            FailMsgToStr(fail_msg),
            fail_code,
            (comm_str ? comm_str : "null"),
            ids_[idx0], center_ticks_[idx0], last_cmd_[idx0], last_now_[idx0], (valid_now_[idx0] ? 1 : 0),
            ids_[idxN], center_ticks_[idxN], last_cmd_[idxN], last_now_[idxN], (valid_now_[idxN] ? 1 : 0)
        );

        // ---------- 프로파일링 통계 (별도 라인) ----------
        int n_dt;   double s_dt,   mn_dt,   mx_dt;
        int n_sr;   double s_sr,   mn_sr,   mx_sr;
        int n_sp;   double s_sp,   mn_sp,   mx_sp;
        int n_bc;   double s_bc,   mn_bc,   mx_bc;
        int n_sw;   double s_sw,   mn_sw,   mx_sw;
        int n_st;   double s_st,   mn_st,   mx_st;
        int n_tot;  double s_tot,  mn_tot,  mx_tot;

        prof_loop_dt_.Consume(n_dt,  s_dt,  mn_dt,  mx_dt);
        prof_sr_txrx_.Consume(n_sr,  s_sr,  mn_sr,  mx_sr);
        prof_sr_parse_.Consume(n_sp, s_sp,  mn_sp,  mx_sp);
        prof_build_cmd_.Consume(n_bc,s_bc,  mn_bc,  mx_bc);
        prof_sw_tx_.Consume(n_sw,    s_sw,  mn_sw,  mx_sw);
        prof_settle_.Consume(n_st,   s_st,  mn_st,  mx_st);
        prof_total_.Consume(n_tot,   s_tot, mn_tot, mx_tot);

        double m_dt_ms,  mn_dt_ms,  mx_dt_ms;
        double m_sr_ms,  mn_sr_ms,  mx_sr_ms;
        double m_sp_ms,  mn_sp_ms,  mx_sp_ms;
        double m_bc_ms,  mn_bc_ms,  mx_bc_ms;
        double m_sw_ms,  mn_sw_ms,  mx_sw_ms;
        double m_st_ms,  mn_st_ms,  mx_st_ms;
        double m_tt_ms,  mn_tt_ms,  mx_tt_ms;

        StatToMs(n_dt,  s_dt,  mn_dt,  mx_dt,  m_dt_ms, mn_dt_ms, mx_dt_ms);
        StatToMs(n_sr,  s_sr,  mn_sr,  mx_sr,  m_sr_ms, mn_sr_ms, mx_sr_ms);
        StatToMs(n_sp,  s_sp,  mn_sp,  mx_sp,  m_sp_ms, mn_sp_ms, mx_sp_ms);
        StatToMs(n_bc,  s_bc,  mn_bc,  mx_bc,  m_bc_ms, mn_bc_ms, mx_bc_ms);
        StatToMs(n_sw,  s_sw,  mn_sw,  mx_sw,  m_sw_ms, mn_sw_ms, mx_sw_ms);
        StatToMs(n_st,  s_st,  mn_st,  mx_st,  m_st_ms, mn_st_ms, mx_st_ms);
        StatToMs(n_tot, s_tot, mn_tot, mx_tot, m_tt_ms, mn_tt_ms, mx_tt_ms);

        RCLCPP_INFO(
            get_logger(),
            "[PROF] n=%d | loop_dt mean=%.6fms(min=%.6f max=%.6f) | "
            "SR(txRx)=%.6fms(min=%.6f max=%.6f) | SR(parse)=%.6fms(min=%.6f max=%.6f) | "
            "build=%.6fms(min=%.6f max=%.6f) | SW(tx)=%.6fms(min=%.6f max=%.6f) | "
            "settle=%.6fms(min=%.6f max=%.6f) | total=%.6fms(min=%.6f max=%.6f)",
            n_tot,
            m_dt_ms, mn_dt_ms, mx_dt_ms,
            m_sr_ms, mn_sr_ms, mx_sr_ms,
            m_sp_ms, mn_sp_ms, mx_sp_ms,
            m_bc_ms, mn_bc_ms, mx_bc_ms,
            m_sw_ms, mn_sw_ms, mx_sw_ms,
            m_st_ms, mn_st_ms, mx_st_ms,
            m_tt_ms, mn_tt_ms, mx_tt_ms
        );
    }

private:
    // Dynamixel
    dynamixel::PortHandler* portHandler_{nullptr};
    dynamixel::PacketHandler* packetHandler_{nullptr};
    dynamixel::GroupSyncWrite* sync_write_{nullptr};
    dynamixel::GroupSyncRead*  sync_read_{nullptr};

    // 설정
    std::string port_name_;
    int baud_;

    int id_first_;
    int id_last_;
    std::vector<int> ids_;
    int motor_n_{0};

    int ADDR_TORQUE_ENABLE;
    int ADDR_GOAL_POSITION;
    int ADDR_PRESENT_POSITION;

    int LEN_GOAL_POSITION;
    int LEN_PRESENT_POSITION;

    double freq_;
    double dt_;

    double kp_;
    int u_limit_;

    // sin
    int amp_;
    double sin_hz_;
    double phase_step_rad_;

    // 안전 tick 범위
    int min_tick_;
    int max_tick_;

    // 모터별 center (ids_와 1:1)
    std::vector<int> center_ticks_;

    int tx_settle_us_;

    // ROS 타이머
    rclcpp::TimerBase::SharedPtr loop_timer_;
    rclcpp::TimerBase::SharedPtr log_timer_;

    // 시간/상태
    time_point<steady_clock> prev_time_;
    double t_sec_{0.0};

    std::atomic<double> last_real_hz_;

    // 모터별 최근값
    std::vector<int> last_ref_;
    std::vector<int> last_now_;
    std::vector<int> last_cmd_;

    // 이번 루프에서 fresh now가 있었는지
    std::vector<bool> valid_now_;

    // 1초 통계 (기존)
    std::atomic<int>    stat_count_;
    std::atomic<double> stat_dt_sum_;
    std::atomic<double> stat_dt_min_;
    std::atomic<double> stat_dt_max_;

    std::atomic<int> read_ok_count_;
    std::atomic<int> read_fail_count_;
    std::atomic<int> write_ok_count_;
    std::atomic<int> write_fail_count_;
    std::atomic<int> avail_miss_count_;

    std::atomic<int> last_fail_code_;
    std::atomic<int> last_fail_msg_;

    // 1초 통계 (프로파일링)
    StageStat prof_loop_dt_;
    StageStat prof_sr_txrx_;
    StageStat prof_sr_parse_;
    StageStat prof_build_cmd_;
    StageStat prof_sw_tx_;
    StageStat prof_settle_;
    StageStat prof_total_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrequencyTestMulti>());
    rclcpp::shutdown();
    return 0;
}
