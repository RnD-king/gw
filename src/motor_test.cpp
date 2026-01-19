#include <chrono>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include "dynamixel_sdk/dynamixel_sdk.h"

using namespace std::chrono;

class MotorTest : public rclcpp::Node
{
public:
    MotorTest() : Node("motor_test")
    {
        port_ = "/dev/ttyUSB0";
        baud_ = 4000000;

        ID_YAW = 21;
        ID_PITCH = 22;

        ADDR_TORQUE_ENABLE = 64;
        ADDR_GOAL_POSITION = 116;
        ADDR_PRESENT_POSITION = 132;

        total_deg = 30.0;
        total_time = 3.0;

        freq = 30.0;
        dt = 1.0 / freq;

        portHandler = dynamixel::PortHandler::getPortHandler(port_.c_str());
        packetHandler = dynamixel::PacketHandler::getPacketHandler(2.0);

        if (!portHandler->openPort())
            RCLCPP_ERROR(get_logger(), "Failed to open port");

        if (!portHandler->setBaudRate(baud_))
            RCLCPP_ERROR(get_logger(), "Failed to set baudrate");

        uint8_t dxl_error = 0;
        packetHandler->write1ByteTxRx(portHandler, ID_YAW, ADDR_TORQUE_ENABLE, 1, &dxl_error);
        packetHandler->write1ByteTxRx(portHandler, ID_PITCH, ADDR_TORQUE_ENABLE, 1, &dxl_error);

        start_yaw = read_pos(ID_YAW);
        start_pitch = read_pos(ID_PITCH);

        start_time = steady_clock::now();
        prev_time_ = start_time;  

        RCLCPP_INFO(get_logger(), "Start Yaw=%d", start_yaw);
        RCLCPP_INFO(get_logger(), "Start Pitch=%d", start_pitch);

        timer = create_wall_timer(
            std::chrono::duration<double>(dt),
            std::bind(&MotorTest::update_loop, this)
        );
    }

private:
    uint32_t read_pos(int id)
    {
        uint8_t error;
        uint32_t pos = 0;
        packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION, &pos, &error);
        return pos;
    }

    int rad_to_pos(double rad)
    {
        double deg = rad * 180.0 / M_PI;
        int pos = int((deg / 360.0) * 4096);
        if (pos < 0) pos = 0;
        if (pos > 4095) pos = 4095;
        return pos;
    }

    void write_pos(int id, int pos)
    {
        uint8_t error;
        packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, pos, &error);
    }

    void update_loop()
    {
        auto now_tp = steady_clock::now();
        double dt_real = duration<double>(now_tp - prev_time_).count();
        prev_time_ = now_tp;
        double real_hz = (dt_real > 0.0) ? 1.0 / dt_real : 0.0;

        double elapsed = duration<double>(steady_clock::now() - start_time).count();

        if (elapsed >= total_time)
        {
            RCLCPP_INFO(get_logger(), "Done.");
            timer->cancel();
            return;
        }

        double progress = elapsed / total_time;
        double target_deg = total_deg * progress;
        double target_rad = target_deg * M_PI / 180.0;

        int tick = rad_to_pos(target_rad);

        int goal_yaw = start_yaw + tick;
        int goal_pitch = start_pitch + tick;

        write_pos(ID_YAW, goal_yaw);
        write_pos(ID_PITCH, goal_pitch);

        uint32_t now_yaw = read_pos(ID_YAW);
        uint32_t now_pitch = read_pos(ID_PITCH);

        RCLCPP_INFO(get_logger(),
            "t=%.2fs Yaw(Goal / Now) %d/%d  Pitch(Goal / Yaw) %d/%d",
            elapsed, goal_yaw, now_yaw, goal_pitch, now_pitch
        );
    }

    dynamixel::PortHandler *portHandler;
    dynamixel::PacketHandler *packetHandler;

    std::string port_;
    int baud_;
    int ID_YAW, ID_PITCH;
    int ADDR_TORQUE_ENABLE, ADDR_GOAL_POSITION, ADDR_PRESENT_POSITION;

    double total_deg, total_time;
    double freq, dt;

    uint32_t start_yaw, start_pitch;

    time_point<steady_clock> start_time;
    time_point<steady_clock> prev_time_; 

    rclcpp::TimerBase::SharedPtr timer;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorTest>());
    rclcpp::shutdown();
    return 0;
}
