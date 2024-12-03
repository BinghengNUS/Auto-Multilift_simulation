#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, SetMode
import math
import time

current_pose = PoseStamped()
current_state = State()

def pose_callback(msg):
    global current_pose
    current_pose = msg

def state_callback(msg):
    global current_state
    current_state = msg

class PIDController:
    def __init__(self, kp, ki, kd, setpoint=0.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.previous_error = 0.0
        self.integral = 0.0

    def compute(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt

        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.previous_error = error

        return output

def main():
    rospy.init_node('pid_position_control_node', anonymous=True)

    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/state', State, state_callback)

    setpoint_pub = rospy.Publisher('/mavros/setpoint_raw/local', PositionTarget, queue_size=10)

    # 服务代理
    arming_client = rospy.ServiceProxy('mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy('mavros/set_mode', SetMode)

    rate = rospy.Rate(20)  # 20Hz

    # 等待连接
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo("等待与飞控连接...")
        rate.sleep()

    # 初始化PID控制器
    pid_x = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=10.0)  # 目标位置x=5.0
    pid_y = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=5.0)  # 目标位置y=5.0
    pid_z = PIDController(kp=1.0, ki=0.0, kd=0.0, setpoint=3.0)  # 目标高度z=2.0

    # 进入OFFBOARD模式前需要先发送一些设定点
    sp = PositionTarget()
    sp.header.stamp = rospy.Time.now()
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    sp.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                   PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                   PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE
    sp.velocity.x = 0.0
    sp.velocity.y = 0.0
    sp.velocity.z = 0.0

    for i in range(100):
        setpoint_pub.publish(sp)
        rate.sleep()

    # 切换模式并解锁
    last_request = rospy.Time.now()

    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            if set_mode_client(custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo("成功切换到OFFBOARD模式")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                if arming_client(True).success:
                    rospy.loginfo("无人机已解锁")
                last_request = rospy.Time.now()

        # 计算控制指令
        dt = 1.0 / 20  # 时间间隔，单位秒

        current_x = current_pose.pose.position.x
        current_y = current_pose.pose.position.y
        current_z = current_pose.pose.position.z

        vel_x = pid_x.compute(current_x, dt)
        vel_y = pid_y.compute(current_y, dt)
        vel_z = pid_z.compute(current_z, dt)

        # 限制速度
        max_vel = 2.0  # 最大速度，单位m/s
        vel_x = max(min(vel_x, max_vel), -max_vel)
        vel_y = max(min(vel_y, max_vel), -max_vel)
        vel_z = max(min(vel_z, max_vel), -max_vel)

        # 发送控制指令
        send_control_command(vel_x, vel_y, vel_z)

        rate.sleep()

def send_control_command(vel_x, vel_y, vel_z):
    sp = PositionTarget()
    sp.header.stamp = rospy.Time.now()
    sp.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
    sp.type_mask = PositionTarget.IGNORE_PX | PositionTarget.IGNORE_PY | PositionTarget.IGNORE_PZ | \
                   PositionTarget.IGNORE_AFX | PositionTarget.IGNORE_AFY | PositionTarget.IGNORE_AFZ | \
                   PositionTarget.IGNORE_YAW | PositionTarget.IGNORE_YAW_RATE

    sp.velocity.x = vel_x
    sp.velocity.y = vel_y
    sp.velocity.z = vel_z

    setpoint_pub.publish(sp)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
