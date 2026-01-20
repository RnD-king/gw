#!/usr/bin/env python3
# 축 안 맞는다 지금
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped

kTicksPerRev = 4096

def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def wrap_to_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


def normalize(v: np.ndarray) -> np.ndarray:
    n = np.linalg.norm(v)
    if n < 1e-9:
        return v
    return v / n


class ImuMahonyNode(Node):
    def __init__(self):
        super().__init__('imu_mahony')

        self.declare_parameter('gyro_topic', '/camera/gyro/sample')
        self.declare_parameter('accel_topic', '/camera/accel/sample')
        self.declare_parameter('gimbal_delta_topic', '/gimbal/delta_tick')

        # Mahony 필터 게인
        self.declare_parameter('Kp', 2.0) # Kp ≈ 2π · f_c  >>>>>>>>>>>>> 많이 흔들리는 상황이면 자이로 신뢰도 높이기
        self.declare_parameter('Ki', 0.0)  # Ki ≈ 0.001  <<<< 굳이 필요 X
        # Gimbal PID/LPF
        self.declare_parameter('cutoff_hz', 15.0)

        self.declare_parameter('kp_pitch', 1.0)
        self.declare_parameter('kd_pitch', 0.01)
        self.declare_parameter('ki_pitch', 0.0)

        self.declare_parameter('kp_yaw', 1.8)
        self.declare_parameter('kd_yaw', 0.02)
        self.declare_parameter('ki_yaw', 0.0)

        self.declare_parameter('max_tick_step_pitch', 200)
        self.declare_parameter('max_tick_step_yaw', 200)

        self.gyro_topic = self.get_parameter('gyro_topic').get_parameter_value().string_value
        self.accel_topic = self.get_parameter('accel_topic').get_parameter_value().string_value
        self.gimbal_delta_topic = self.get_parameter('gimbal_delta_topic').get_parameter_value().string_value

        self.Kp = float(self.get_parameter('Kp').value)
        self.Ki = float(self.get_parameter('Ki').value)

        self.cutoff_hz = float(self.get_parameter('cutoff_hz').value)
        self.kp_pitch = float(self.get_parameter('kp_pitch').value)
        self.kd_pitch = float(self.get_parameter('kd_pitch').value)
        self.ki_pitch = float(self.get_parameter('ki_pitch').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.max_tick_step_pitch = max(1, int(self.get_parameter('max_tick_step_pitch').value))
        self.max_tick_step_yaw = max(1, int(self.get_parameter('max_tick_step_yaw').value))

        self.use_initial_ref = True
        self.ref_set = False
        self.roll_ref = 0.0
        self.pitch_ref = 0.0
        self.yaw_ref = 0.0

        self.last_gyro: Imu | None = None
        self.last_accel: Imu | None = None

        self.last_time = None
        
        self.q = np.array([1.0, 0.0, 0.0, 0.0], dtype=float) # quaternion

        self.gyro_bias = np.zeros(3, dtype=float)
        self.integral_error = np.zeros(3, dtype=float)

        self.calibrating = True
        self.calib_start_time = None
        self.gyro_bias_sum = np.zeros(3, dtype=float)
        self.bias_samples = 0

        self.last_roll_out = 0.0
        self.last_pitch_out = 0.0
        self.last_yaw_out = 0.0
        self.pid_initialized = False
        self.err_pitch_lpf = 0.0
        self.err_yaw_lpf = 0.0
        self.prev_pitch_out = 0.0
        self.prev_yaw_out = 0.0
        self.err_pitch_integral = 0.0
        self.err_yaw_integral = 0.0
        self.loop_count = 0
        self.loop_dt_accum = 0.0

        self.sub_gyro = self.create_subscription(Imu, self.gyro_topic, self.gyro_callback, qos_profile_sensor_data)
        self.sub_accel = self.create_subscription(Imu, self.accel_topic, self.accel_callback, qos_profile_sensor_data)

        self.pub_tilt = self.create_publisher(Vector3Stamped, '/gimbal/tilt_rad', 10)
        self.pub_delta = self.create_publisher(Vector3Stamped, self.gimbal_delta_topic, 10)

        self.timer = self.create_timer(1/200, self.update_filter)

        self.debug_timer = self.create_timer(1.0, self.debug_print) # 출력은 1초마다

        self.get_logger().info(
            f'Mahony IMU node started. gyro_topic={self.gyro_topic}, '
            f'accel_topic={self.accel_topic}, Kp={self.Kp:.3f}, Ki={self.Ki:.3f}, '
            f'gimbal_delta_topic={self.gimbal_delta_topic}'
        )
        self.get_logger().info(
            f'Gimbal PID: cutoff_hz={self.cutoff_hz:.2f}, '
            f'kp/ki/kd pitch={self.kp_pitch:.3f}/{self.ki_pitch:.3f}/{self.kd_pitch:.3f}, '
            f'kp/ki/kd yaw={self.kp_yaw:.3f}/{self.ki_yaw:.3f}/{self.kd_yaw:.3f}, '
            f'max_tick_step(P/Y)={self.max_tick_step_pitch}/{self.max_tick_step_yaw}'
        )

    def gyro_callback(self, msg: Imu):
        self.last_gyro = msg
        if self.calibrating and self.calib_start_time is None: # 자이로 캘리브레이션
            self.calib_start_time = self.get_clock().now()

    def accel_callback(self, msg: Imu):
        self.last_accel = msg

    def debug_print(self):
        roll_deg = math.degrees(self.last_roll_out)
        pitch_deg = math.degrees(self.last_pitch_out)
        yaw_deg = math.degrees(self.last_yaw_out)

        if self.loop_count > 0 and self.loop_dt_accum > 0.0:
            loop_hz = self.loop_count / self.loop_dt_accum
            self.get_logger().info(f"[Mahony loop] rate={loop_hz:.1f} Hz")
            self.loop_count = 0
            self.loop_dt_accum = 0.0

        self.get_logger().info(
            "[Mahony angle vs ref] "
            f"roll={self.last_roll_out:+.4f} rad ({roll_deg:+.2f} deg), "
            f"pitch={self.last_pitch_out:+.4f} rad ({pitch_deg:+.2f} deg), "
            f"yaw={self.last_yaw_out:+.4f} rad ({yaw_deg:+.2f} deg)"
        )

    def update_filter(self):
        if self.last_gyro is None or self.last_accel is None: # gyro/accel 둘 다 준비 안되면 대기
            return
        
        now = self.get_clock().now()
        if self.last_time is None:
            self.last_time = now
            return

        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0 or dt > 0.1:
            return
        self.loop_count += 1
        self.loop_dt_accum += dt

        gx = self.last_gyro.angular_velocity.x
        gy = self.last_gyro.angular_velocity.y
        gz = self.last_gyro.angular_velocity.z

        ax = self.last_accel.linear_acceleration.x
        ay = self.last_accel.linear_acceleration.y
        az = self.last_accel.linear_acceleration.z

        gyro = np.array([gx, gy, gz], dtype=float)
        accel = np.array([ax, ay, az], dtype=float)

        if self.calibrating: # 1. 캘리브레이션
            if self.calib_start_time is None:
                return

            elapsed = (now - self.calib_start_time).nanoseconds * 1e-9
            if elapsed < 5.0:
                self.gyro_bias_sum += gyro
                self.bias_samples += 1
                return
            else:
                if self.bias_samples > 0:
                    self.gyro_bias = self.gyro_bias_sum / self.bias_samples
                else:
                    self.gyro_bias = np.zeros(3, dtype=float)
                    self.get_logger().warn('No samples collected for gyro bias calibration.')

                self.calibrating = False
                self.integral_error[:] = 0.0
                self.get_logger().info(f'[Mahony] Initial gyro bias = {self.gyro_bias}')

                roll0 = math.atan2(ay, az) # accel 기반 초기 기울기 계산
                pitch0 = math.atan2(-ax, math.sqrt(ay*ay + az*az))

                self.roll_ref = roll0
                self.pitch_ref = pitch0
                self.yaw_ref = 0.0  # yaw는 accel로 계산 불가 >>>>>>>>>> 0으로 기준

                self.ref_set = True
                self.get_logger().info(
                    f"[Mahony] Reference orientation set immediately: "
                    f"roll_ref={roll0:.3f}, pitch_ref={pitch0:.3f}, yaw_ref={self.yaw_ref:.3f}"
                )
                return

        gyro_unbiased = gyro - self.gyro_bias # 2. Mahony 필터 업뎃

        accel_norm = np.linalg.norm(accel) # 정규화
        if accel_norm < 1e-6:
            accel_unit = None
        else:
            accel_unit = accel / accel_norm

        q0, q1, q2, q3 = self.q

        if accel_unit is not None: # 가속도계 오차 보정
            vx = 2.0 * (q1 * q3 - q0 * q2)
            vy = 2.0 * (q0 * q1 + q2 * q3)
            vz = (q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3)

            ax_u, ay_u, az_u = accel_unit  # 오차 = accel_unit × v
            ex = (ay_u * vz - az_u * vy)
            ey = (az_u * vx - ax_u * vz)
            ez = (ax_u * vy - ay_u * vx)
            error = np.array([ex, ey, ez], dtype=float)

            if self.Ki > 0.0:
                self.integral_error += error * dt
                gyro_corr = gyro_unbiased + self.Kp * error + self.Ki * self.integral_error
            else:
                gyro_corr = gyro_unbiased + self.Kp * error
        else:
            gyro_corr = gyro_unbiased

        gx_c, gy_c, gz_c = gyro_corr

        # quaternion 적분
        q_dot0 = 0.5 * (-q1 * gx_c - q2 * gy_c - q3 * gz_c)
        q_dot1 = 0.5 * ( q0 * gx_c + q2 * gz_c - q3 * gy_c)
        q_dot2 = 0.5 * ( q0 * gy_c - q1 * gz_c + q3 * gx_c)
        q_dot3 = 0.5 * ( q0 * gz_c + q1 * gy_c - q2 * gx_c)

        q0 += q_dot0 * dt
        q1 += q_dot1 * dt
        q2 += q_dot2 * dt
        q3 += q_dot3 * dt

        q_new = normalize(np.array([q0, q1, q2, q3], dtype=float))
        self.q = q_new
        q0, q1, q2, q3 = q_new

        sinr_cosp = 2.0 * (q0 * q1 + q2 * q3) # 3. 쿼터니안 >>> 오일러 각 변환
        cosr_cosp = 1.0 - 2.0 * (q1 * q1 + q2 * q2)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (q0 * q2 - q3 * q1)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (q0 * q3 + q1 * q2)
        cosy_cosp = 1.0 - 2.0 * (q2 * q2 + q3 * q3)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        if self.use_initial_ref and not self.ref_set: # 최초 1회 기준각 설정
            self.roll_ref = roll
            self.pitch_ref = pitch
            self.yaw_ref = yaw
            self.ref_set = True
            self.get_logger().info(
                f"[Mahony] Reference orientation set: "
                f"roll_ref={self.roll_ref:.3f}, pitch_ref={self.pitch_ref:.3f}, yaw_ref={self.yaw_ref:.3f}"
            )

        if self.ref_set:
            roll_raw = roll - self.roll_ref
            pitch_raw = pitch - self.pitch_ref
            yaw_raw = yaw - self.yaw_ref
        else:
            roll_raw, pitch_raw, yaw_raw = roll, pitch, yaw

        self.last_roll_out = roll_raw
        self.last_pitch_out = pitch_raw
        self.last_yaw_out = yaw_raw

        msg_tilt = Vector3Stamped() # 4. 결과 퍼블리시  >>>>>  초기각 기준 틀어진 각도
        msg_tilt.header.stamp = now.to_msg()
        msg_tilt.header.frame_id = 'camera_link'
        msg_tilt.vector.x = roll_raw
        msg_tilt.vector.y = pitch_raw
        msg_tilt.vector.z = yaw_raw
        self.pub_tilt.publish(msg_tilt)

        if not self.ref_set:
            return

        if not self.pid_initialized:
            self.prev_pitch_out = pitch_raw
            self.prev_yaw_out = yaw_raw
            self.err_pitch_lpf = 0.0
            self.err_yaw_lpf = 0.0
            self.err_pitch_integral = 0.0
            self.err_yaw_integral = 0.0
            self.pid_initialized = True
            return

        err_pitch = roll_raw
        err_yaw = wrap_to_pi(yaw_raw)

        if self.cutoff_hz > 0.0:
            rc = 1.0 / (2.0 * math.pi * self.cutoff_hz)
            a = dt / (rc + dt)
            self.err_pitch_lpf += a * (err_pitch - self.err_pitch_lpf)
            self.err_yaw_lpf += a * (err_yaw - self.err_yaw_lpf)
        else:
            self.err_pitch_lpf = err_pitch
            self.err_yaw_lpf = err_yaw

        pitch_rate = (pitch_raw - self.prev_pitch_out) / dt
        yaw_rate = wrap_to_pi(yaw_raw - self.prev_yaw_out) / dt
        self.prev_pitch_out = pitch_raw
        self.prev_yaw_out = yaw_raw

        if self.ki_pitch > 0.0:
            self.err_pitch_integral += self.err_pitch_lpf * dt
        else:
            self.err_pitch_integral = 0.0

        if self.ki_yaw > 0.0:
            self.err_yaw_integral += self.err_yaw_lpf * dt
        else:
            self.err_yaw_integral = 0.0

        ticks_per_rad = float(kTicksPerRev) / (2.0 * math.pi)
        if self.ki_pitch > 0.0:
            max_pitch_integral = (self.max_tick_step_pitch / ticks_per_rad) / self.ki_pitch
            self.err_pitch_integral = clamp(self.err_pitch_integral, -max_pitch_integral, +max_pitch_integral)
        if self.ki_yaw > 0.0:
            max_yaw_integral = (self.max_tick_step_yaw / ticks_per_rad) / self.ki_yaw
            self.err_yaw_integral = clamp(self.err_yaw_integral, -max_yaw_integral, +max_yaw_integral)

        cmd_pitch_rad = -(
            self.kp_pitch * self.err_pitch_lpf
            + self.kd_pitch * pitch_rate
            + self.ki_pitch * self.err_pitch_integral
        )
        cmd_yaw_rad = -(
            self.kp_yaw * self.err_yaw_lpf
            + self.kd_yaw * yaw_rate
            + self.ki_yaw * self.err_yaw_integral
        )

        delta_pitch_tick = int(round(cmd_pitch_rad * ticks_per_rad))
        delta_yaw_tick = int(round(cmd_yaw_rad * ticks_per_rad))

        delta_pitch_tick = int(clamp(delta_pitch_tick, -self.max_tick_step_pitch, +self.max_tick_step_pitch))
        delta_yaw_tick = int(clamp(delta_yaw_tick, -self.max_tick_step_yaw, +self.max_tick_step_yaw))

        msg_delta = Vector3Stamped()
        msg_delta.header.stamp = now.to_msg()
        msg_delta.header.frame_id = 'camera_link'
        msg_delta.vector.x = 0.0
        msg_delta.vector.y = float(delta_pitch_tick)
        msg_delta.vector.z = float(delta_yaw_tick)
        self.pub_delta.publish(msg_delta)


def main():
    rclpy.init()
    node = ImuMahonyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
