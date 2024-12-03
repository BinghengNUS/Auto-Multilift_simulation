#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import time
import math
from std_msgs.msg import Float64
from geometry_msgs.msg import Pose, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import tf

# 定义 PID 控制器类
class PIDController:
    def __init__(self, kp, ki, kd, max_output, min_output):
        self.kp = kp  # 比例系数
        self.ki = ki  # 积分系数
        self.kd = kd  # 微分系数
        self.max_output = max_output  # 输出上限
        self.min_output = min_output  # 输出下限
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = (self.kp * error) + (self.ki * self.integral) + (self.kd * derivative)
        output = max(min(output, self.max_output), self.min_output)
        self.prev_error = error
        return output

# 全局变量
current_pose = Pose()
current_twist = Twist()
current_imu = Imu()

def odometry_callback(msg):
    global current_pose, current_twist
    current_pose = msg.pose.pose
    current_twist = msg.twist.twist

def imu_callback(msg):
    global current_imu
    current_imu = msg

def quaternion_to_euler(quaternion):
    euler = tf.transformations.euler_from_quaternion([
        quaternion.x, quaternion.y, quaternion.z, quaternion.w
    ])
    return euler  # 返回 (roll, pitch, yaw)

def mixer(roll, pitch, yaw, thrust):
    # 定义最大和最小电机转速
    MAX_MOTOR_SPEED = 1000.0
    MIN_MOTOR_SPEED = 0.0

    # 简单的混合器，根据姿态控制输出和推力计算每个电机的转速
    motor1 = thrust + roll + pitch - yaw  # 前右电机
    motor2 = thrust - roll + pitch + yaw  # 后左电机
    motor3 = thrust - roll - pitch - yaw  # 前左电机
    motor4 = thrust + roll - pitch + yaw  # 后右电机

    motor_speeds = [motor1, motor2, motor3, motor4]
    motor_speeds = [
        max(min(speed, MAX_MOTOR_SPEED), MIN_MOTOR_SPEED) for speed in motor_speeds
    ]
    return motor_speeds

def publish_motor_commands(motor_speeds, motor_pub):
    for i in range(4):
        motor_pub[i].publish(Float64(motor_speeds[i]))

def main():
    rospy.init_node('drone_controller')
    rate = rospy.Rate(100)  # 控制频率 100 Hz

    # 订阅无人机的里程计信息和 IMU 数据
    rospy.Subscriber('/my_drone/odom', Odometry, odometry_callback)
    rospy.Subscriber('/my_drone/imu', Imu, imu_callback)

    # 发布电机速度指令的发布者
    motor_pub = [
        rospy.Publisher('/my_drone/rotor_0_joint_controller/command', Float64, queue_size=1),
        rospy.Publisher('/my_drone/rotor_1_joint_controller/command', Float64, queue_size=1),
        rospy.Publisher('/my_drone/rotor_2_joint_controller/command', Float64, queue_size=1),
        rospy.Publisher('/my_drone/rotor_3_joint_controller/command', Float64, queue_size=1)
    ]

    # 初始化 PID 控制器
    # 外环 PID 控制器（位置控制）
    pid_x = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=0.2, min_output=-0.2)
    pid_y = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=0.2, min_output=-0.2)
    pid_z = PIDController(kp=5.0, ki=0.0, kd=0.2, max_output=15.0, min_output=0.0)

    # 内环 PID 控制器（姿态控制）
    pid_roll = PIDController(kp=6.0, ki=0.0, kd=0.3, max_output=2.0, min_output=-2.0)
    pid_pitch = PIDController(kp=6.0, ki=0.0, kd=0.3, max_output=2.0, min_output=-2.0)
    pid_yaw = PIDController(kp=1.0, ki=0.0, kd=0.1, max_output=1.0, min_output=-1.0)

    # 目标位置和姿态
    target_position = [0.0, 0.0, 5.0]  # 期望位置（x, y, z）
    target_yaw = 0.0  # 期望航向角

    last_time = time.time()

    while not rospy.is_shutdown():
        current_time = time.time()
        dt = current_time - last_time
        last_time = current_time

        # 获取当前位置和姿态
        pos = current_pose.position
        orientation = current_pose.orientation

        # 计算位置误差
        error_x = target_position[0] - pos.x
        error_y = target_position[1] - pos.y
        error_z = target_position[2] - pos.z

        # 外环 PID 控制，生成期望的姿态角（roll 和 pitch）
        desired_roll = pid_y.compute(error_y, dt)  # 注意坐标轴方向
        desired_pitch = pid_x.compute(error_x, dt)
        desired_thrust = pid_z.compute(error_z, dt)

        # 获取当前姿态角（从四元数转换）
        current_roll, current_pitch, current_yaw = quaternion_to_euler(orientation)

        # 计算姿态误差
        error_roll = desired_roll - current_roll
        error_pitch = desired_pitch - current_pitch
        error_yaw = target_yaw - current_yaw

        # 确保航向角误差在 [-pi, pi] 范围内
        error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi

        # 内环 PID 控制，生成姿态控制输出
        control_roll = pid_roll.compute(error_roll, dt)
        control_pitch = pid_pitch.compute(error_pitch, dt)
        control_yaw = pid_yaw.compute(error_yaw, dt)

        # 混合器，将姿态控制输出和推力转换为电机速度指令
        motor_speeds = mixer(control_roll, control_pitch, control_yaw, desired_thrust)

        # 发布电机速度指令
        publish_motor_commands(motor_speeds, motor_pub)

        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
