#!/usr/bin/env python3
import math
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import Imu
from geometry_msgs.msg import Vector3Stamped


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

        # Mahony 필터 게인
        self.declare_parameter('Kp', 2.0) # Kp ≈ 2π · f_c  >>>>>>>>>>>>> 많이 흔들리는 상황이면 자이로 신뢰도 높이기
        self.declare_parameter('Ki', 0.0)  # Ki ≈ 0.001  <<<< 굳이 필요 X

        self.gyro_topic = self.get_parameter('gyro_topic').get_parameter_value().string_value
        self.accel_topic = self.get_parameter('accel_topic').get_parameter_value().string_value

        self.Kp = float(self.get_parameter('Kp').value)
        self.Ki = float(self.get_parameter('Ki').value)

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

        self.sub_gyro = self.create_subscription(Imu, self.gyro_topic, self.gyro_callback, qos_profile_sensor_data)
        self.sub_accel = self.create_subscription(Imu, self.accel_topic, self.accel_callback, qos_profile_sensor_data)

        self.pub_tilt = self.create_publisher(Vector3Stamped, '/camera/imu_tilt', 10)

        self.timer = self.create_timer(1/200, self.update_filter)

        self.debug_timer = self.create_timer(1.0, self.debug_print) # 출력은 1초마다

        self.get_logger().info(
            f'Mahony IMU node started. gyro_topic={self.gyro_topic}, '
            f'accel_topic={self.accel_topic}, Kp={self.Kp:.3f}, Ki={self.Ki:.3f}'
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
            roll_out = roll - self.roll_ref
            pitch_out = pitch - self.pitch_ref
            yaw_out = yaw - self.yaw_ref
        else:
            roll_out, pitch_out, yaw_out = roll, pitch, yaw

        self.last_roll_out = roll_out
        self.last_pitch_out = pitch_out
        self.last_yaw_out = yaw_out

        msg_tilt = Vector3Stamped() # 4. 결과 퍼블리시  >>>>>  초기각 기준 틀어진 각도
        msg_tilt.header.stamp = now.to_msg()
        msg_tilt.header.frame_id = 'camera_link'
        msg_tilt.vector.x = roll_out
        msg_tilt.vector.y = pitch_out
        msg_tilt.vector.z = yaw_out
        self.pub_tilt.publish(msg_tilt)


def main():
    rclpy.init()
    node = ImuMahonyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

