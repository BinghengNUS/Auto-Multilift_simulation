#!/usr/bin/env python3

import rospy
import threading
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from std_msgs.msg import Header

# 全局状态字典，存储各无人机的状态
uav_states = {}
uav_poses = {}

def state_callback(uav_ns, msg):
    global uav_states
    uav_states[uav_ns] = msg
    rospy.loginfo(f"{uav_ns} state updated: connected={msg.connected}, armed={msg.armed}, mode={msg.mode}")

def pose_callback(uav_ns, msg):
    global uav_poses
    uav_poses[uav_ns] = msg.pose
    rospy.loginfo(f"{uav_ns} position updated: x={msg.pose.position.x}, y={msg.pose.position.y}, z={msg.pose.position.z}")

def takeoff_hover(uav_ns, condition, ready_count, takeoff_time):
    rospy.loginfo(f"Starting takeoff sequence for {uav_ns}")

    # 初始化状态
    current_state = None
    current_pose = None

    # 订阅无人机的状态和位置
    rospy.Subscriber(f"/{uav_ns}/mavros/state", State, lambda msg: state_callback(uav_ns, msg))
    rospy.Subscriber(f"/{uav_ns}/mavros/local_position/pose", PoseStamped, lambda msg: pose_callback(uav_ns, msg))

    # 创建发布者
    setpoint_pub = rospy.Publisher(f'/{uav_ns}/mavros/setpoint_position/local', PoseStamped, queue_size=10)

    # 等待 FCU 连接
    rate = rospy.Rate(20.0)  # 20Hz
    while not rospy.is_shutdown():
        if uav_ns in uav_states and uav_states[uav_ns].connected:
            rospy.loginfo(f"{uav_ns} connected to FCU")
            break
        else:
            rospy.loginfo(f"Waiting for FCU connection on {uav_ns}...")
            rate.sleep()

    # 等待服务可用
    rospy.wait_for_service(f'/{uav_ns}/mavros/cmd/arming')
    rospy.wait_for_service(f'/{uav_ns}/mavros/set_mode')

    # 创建服务代理
    arming_client = rospy.ServiceProxy(f'/{uav_ns}/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy(f'/{uav_ns}/mavros/set_mode', SetMode)

    # 等待获取当前位置
    while not rospy.is_shutdown():
        if uav_ns in uav_poses:
            rospy.loginfo(f"{uav_ns} received current position")
            break
        else:
            rospy.loginfo(f"Waiting for initial position on {uav_ns}...")
            rate.sleep()

    # 创建目标位置，飞到当前 x, y, z+2
    pose = PoseStamped()
    pose.header = Header()
    pose.header.stamp = rospy.Time.now()
    pose.pose.position.x = uav_poses[uav_ns].position.x
    pose.pose.position.y = uav_poses[uav_ns].position.y
    pose.pose.position.z = uav_poses[uav_ns].position.z + 2

    # 在切换模式之前发送一些 setpoint
    rospy.loginfo(f"{uav_ns} sending initial setpoints for mode change preparation")
    for i in range(100):
        setpoint_pub.publish(pose)
        rate.sleep()

    # 通知主线程该无人机已准备就绪
    with condition:
        ready_count[0] += 1
        rospy.loginfo(f"{uav_ns} is ready ({ready_count[0]}/6)")

        # 如果达到至少6架无人机准备就绪，并且还没有设置起飞时间
        if ready_count[0] >= 6 and takeoff_time[0] is None:
            takeoff_time[0] = rospy.Time.now() + rospy.Duration(5.0)
            rospy.loginfo(f"Takeoff time set to {takeoff_time[0].to_sec()} (current time {rospy.Time.now().to_sec()})")
            condition.notify_all()
        else:
            # 如果起飞时间已设置，通知其他线程
            condition.notify_all()

        # 如果起飞时间还未设置，等待
        while takeoff_time[0] is None:
            rospy.loginfo(f"{uav_ns} waiting for takeoff time")
            condition.wait()

    # 等待仿真时间达到指定的起飞时间
    rospy.loginfo(f"{uav_ns} waiting until takeoff time")
    while not rospy.is_shutdown() and rospy.Time.now() < takeoff_time[0]:
        setpoint_pub.publish(pose)
        rate.sleep()

    # 设置 OFFBOARD 模式并解锁
    last_request = rospy.Time.now()
    while not rospy.is_shutdown():
        current_state = uav_states.get(uav_ns)
        if current_state is None:
            rospy.logwarn(f"{uav_ns} state is not available")
            rate.sleep()
            continue

        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            res = set_mode_client(base_mode=0, custom_mode="OFFBOARD")
            if res.mode_sent:
                rospy.loginfo(f"{uav_ns} OFFBOARD enabled")
            else:
                rospy.logwarn(f"{uav_ns} failed to set OFFBOARD mode")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                res = arming_client(True)
                if res.success:
                    rospy.loginfo(f"{uav_ns} armed")
                else:
                    rospy.logwarn(f"{uav_ns} failed to arm")
                last_request = rospy.Time.now()

        setpoint_pub.publish(pose)
        rate.sleep()

        # 检查是否已经进入 OFFBOARD 模式并解锁
        if current_state.mode == "OFFBOARD" and current_state.armed:
            rospy.loginfo(f"{uav_ns} is in OFFBOARD mode and armed")
            break

    # 持续发布 setpoint，保持悬停
    rospy.loginfo(f"{uav_ns} holding hover position")
    while not rospy.is_shutdown():
        pose.header.stamp = rospy.Time.now()
        setpoint_pub.publish(pose)
        rate.sleep()

def main():
    rospy.init_node('multi_uav_takeoff_hover', anonymous=True)

    uav_list = ['uav1', 'uav2', 'uav3', 'uav4', 'uav5', 'uav6']

    condition = threading.Condition()
    ready_count = [0]
    takeoff_time = [None]

    threads = []
    for uav in uav_list:
        t = threading.Thread(target=takeoff_hover, args=(uav, condition, ready_count, takeoff_time))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

if __name__ == '__main__':
    main()
