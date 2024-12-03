#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64
from gazebo_msgs.msg import ModelStates
from sensor_msgs.msg import Imu
import tf
import math

class PIDController:
    def __init__(self, kp, ki, kd, max_output, min_output):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        self.integral = 0
        self.prev_error = 0

    def compute(self, error, dt):
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(min(output, self.max_output), self.min_output)
        self.prev_error = error
        return output

class DroneController:
    def __init__(self):
        rospy.init_node('drone_controller')

        # 初始化订阅
        rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        rospy.Subscriber('/my_drone/imu', Imu, self.imu_callback)

        # 初始化发布
        self.rotor_pubs = [
            rospy.Publisher('/my_drone/rotor_0_joint_controller/command', Float64, queue_size=1),
            rospy.Publisher('/my_drone/rotor_1_joint_controller/command', Float64, queue_size=1),
            rospy.Publisher('/my_drone/rotor_2_joint_controller/command', Float64, queue_size=1),
            rospy.Publisher('/my_drone/rotor_3_joint_controller/command', Float64, queue_size=1)
        ]

        # 外环
        self.pid_x = PIDController(kp=10.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0)
        self.pid_y = PIDController(kp=10.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0)
        self.pid_z = PIDController(kp=100.0, ki=0.0, kd=0.0, max_output=100.0, min_output=0.0)

        # 内环
        self.pid_roll = PIDController(kp=6.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0)
        self.pid_pitch = PIDController(kp=6.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0)
        self.pid_yaw = PIDController(kp=1.0, ki=0.0, kd=0.0, max_output=10.0, min_output=-10.0)

        # 目标
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = 5.0
        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

        # 当前状态
        self.current_pose = None
        self.current_imu = None

        # 时间
        self.last_time = rospy.Time.now()

    def model_states_callback(self, msg):
        try:
            index = msg.name.index('my_drone')
            self.current_pose = msg.pose[index]
            # self.current_vel = msg.twist[index]   # 速度
        except ValueError:
            rospy.logwarn_throttle(5, "Model 'my_drone' not found in /gazebo/model_states")

    def imu_callback(self, msg):
        self.current_imu = msg

    def quaternion_to_euler(self, quaternion):
        euler = tf.transformations.euler_from_quaternion([
            quaternion.x, quaternion.y, quaternion.z, quaternion.w
        ])
        return euler  # (roll, pitch, yaw)

    def mixer(self, control_roll, control_pitch, control_yaw, thrust):
        effort1 = thrust + control_roll + control_pitch - control_yaw
        effort2 = thrust - control_roll + control_pitch + control_yaw
        effort3 = thrust - control_roll - control_pitch - control_yaw
        effort4 = thrust + control_roll - control_pitch + control_yaw
        max_effort = 100.0
        min_effort = 0.0
        efforts = [effort1, effort2, effort3, effort4]
        efforts = [max(min(effort, max_effort), min_effort) for effort in efforts]

        return efforts

    def run(self):
        rate = rospy.Rate(1000)  # 1000 Hz
        while not rospy.is_shutdown():
            if self.current_pose is None:
                rospy.logwarn_throttle(1, "Waiting for /gazebo/model_states with 'my_drone'")
                rate.sleep()
                continue

            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            if dt == 0.0:
                rate.sleep()
                continue
            self.last_time = current_time

            # 获取当前姿态和位置
            pos = self.current_pose.position
            orientation = self.current_pose.orientation

            # 计算位置误差
            error_x = self.target_x - pos.x
            error_y = self.target_y - pos.y
            error_z = self.target_z - pos.z

            # 外环 PID 控制，生成期望的姿态角和推力
            desired_roll = self.pid_y.compute(error_y, dt)
            desired_pitch = self.pid_x.compute(error_x, dt)
            desired_thrust = self.pid_z.compute(error_z, dt)

            # 当前姿态角
            current_roll, current_pitch, current_yaw = self.quaternion_to_euler(orientation)

            # 计算姿态误差
            error_roll = desired_roll - current_roll
            error_pitch = desired_pitch - current_pitch
            error_yaw = self.target_yaw - current_yaw
            # 规范化航向角误差到 [-pi, pi]
            error_yaw = (error_yaw + math.pi) % (2 * math.pi) - math.pi

            # 内环 PID 控制，生成姿态控制输出
            control_roll = self.pid_roll.compute(error_roll, dt)
            control_pitch = self.pid_pitch.compute(error_pitch, dt)
            control_yaw = self.pid_yaw.compute(error_yaw, dt)

            motor_efforts = self.mixer(control_roll, control_pitch, control_yaw, desired_thrust)

            # 发布
            for i in range(4):
                self.rotor_pubs[i].publish(Float64(motor_efforts[i]))

            rate.sleep()

if __name__ == '__main__':
    try:
        controller = DroneController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
