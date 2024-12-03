#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Imu
from geometry_msgs.msg import PoseStamped
import math

class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0

    def compute(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        return output

class DroneController:
    def __init__(self):
        rospy.init_node('pid_controller')

        # 订阅无人机状态信息
        rospy.Subscriber('/my_drone/pose', PoseStamped, self.pose_callback)
        rospy.Subscriber('/my_drone/imu', Imu, self.imu_callback)

        # 发布电机控制命令
        self.rotor_0_pub = rospy.Publisher('/my_drone/rotor_0_joint_controller/command', Float64, queue_size=1)
        self.rotor_1_pub = rospy.Publisher('/my_drone/rotor_1_joint_controller/command', Float64, queue_size=1)
        # ... 其他电机

        # 初始化 PID 控制器
        self.altitude_controller = PIDController(kp=1.0, ki=0.0, kd=0.1)
        # ... 其他控制器

        self.current_pose = PoseStamped()
        self.current_imu = Imu()

    def pose_callback(self, msg):
        self.current_pose = msg

    def imu_callback(self, msg):
        self.current_imu = msg

    def run(self):
        rate = rospy.Rate(100)  # 100Hz
        while not rospy.is_shutdown():
            dt = 0.01  # 时间步长，单位秒
            # 获取当前高度
            current_altitude = self.current_pose.pose.position.z
            # 设置目标高度
            desired_altitude = 2.0  # 目标高度 2 米

            # 计算控制输出
            altitude_control = self.altitude_controller.compute(desired_altitude, current_altitude, dt)

            # 发布电机控制命令
            self.rotor_0_pub.publish(altitude_control)
            self.rotor_1_pub.publish(altitude_control)
            # ... 其他电机

            rate.sleep()

if __name__ == '__main__':
    controller = DroneController()
    controller.run()
