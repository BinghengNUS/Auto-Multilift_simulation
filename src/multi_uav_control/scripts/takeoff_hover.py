#!/usr/bin/env python

import rospy
import threading
from mavros_msgs.srv import CommandBool, SetMode
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State

def takeoff_hover(uav_ns, condition, ready_count, takeoff_time):
    rospy.loginfo(f"Starting takeoff sequence for {uav_ns}")

    current_state = State()
    def state_cb(msg):
        nonlocal current_state
        current_state = msg

    rospy.Subscriber(f"/{uav_ns}/mavros/state", State, state_cb)
    rospy.loginfo(f"Subscribed to /{uav_ns}/mavros/state")

    setpoint_pub = rospy.Publisher(f'/{uav_ns}/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rospy.loginfo(f"Publisher created for /{uav_ns}/mavros/setpoint_position/local")

    rate = rospy.Rate(20.0)  # 20Hz
    while not rospy.is_shutdown() and not current_state.connected:
        rospy.loginfo(f"Waiting for FCU connection on {uav_ns}...")
        rate.sleep()

    rospy.loginfo(f"FCU connected for {uav_ns}")

    rospy.wait_for_service(f'/{uav_ns}/mavros/cmd/arming')
    rospy.wait_for_service(f'/{uav_ns}/mavros/set_mode')
    rospy.loginfo(f"Services available for {uav_ns}")

    arming_client = rospy.ServiceProxy(f'/{uav_ns}/mavros/cmd/arming', CommandBool)
    set_mode_client = rospy.ServiceProxy(f'/{uav_ns}/mavros/set_mode', SetMode)

    current_pose = None
    def pose_cb(msg):
        nonlocal current_pose
        current_pose = msg.pose

    rospy.Subscriber(f"/{uav_ns}/mavros/local_position/pose", PoseStamped, pose_cb)
    rospy.loginfo(f"Subscribed to /{uav_ns}/mavros/local_position/pose")

    while not rospy.is_shutdown() and current_pose is None:
        rospy.loginfo(f"Waiting for initial position on {uav_ns}...")
        rate.sleep()

    rospy.loginfo(f"Initial position received for {uav_ns}")

    pose = PoseStamped()
    pose.pose.position.x = current_pose.position.x
    pose.pose.position.y = current_pose.position.y
    pose.pose.position.z = current_pose.position.z + 2
    rospy.loginfo(f"Setpoint position set for {uav_ns}: {pose.pose.position}")

    for i in range(100):
        setpoint_pub.publish(pose)
        rate.sleep()

    rospy.loginfo(f"Setpoint published 100 times for {uav_ns}")

    with condition:
        ready_count[0] += 1
        rospy.loginfo(f"{uav_ns} is ready ({ready_count[0]}/6)")

        if ready_count[0] >= 5 and takeoff_time[0] is None:
            takeoff_time[0] = rospy.Time.now() + rospy.Duration(5.0)
            rospy.loginfo(f"Takeoff time set to {takeoff_time[0].to_sec()} (current time {rospy.Time.now().to_sec()})")
            condition.notify_all()
        else:
            condition.notify_all()

        while takeoff_time[0] is None:
            condition.wait()

    rospy.loginfo(f"Waiting until takeoff time for {uav_ns}")
    while not rospy.is_shutdown() and rospy.Time.now() < takeoff_time[0]:
        setpoint_pub.publish(pose)
        rate.sleep()

    rospy.loginfo(f"Takeoff time reached for {uav_ns}, setting OFFBOARD and arming")

    last_request = rospy.Time.now()
    while not rospy.is_shutdown():
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
            if set_mode_client(base_mode=0, custom_mode="OFFBOARD").mode_sent:
                rospy.loginfo(f"{uav_ns} Offboard enabled")
            else:
                rospy.logwarn(f"{uav_ns} failed to set OFFBOARD mode")
            last_request = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                if arming_client(True).success:
                    rospy.loginfo(f"{uav_ns} armed")
                else:
                    rospy.logwarn(f"{uav_ns} failed to arm")
                last_request = rospy.Time.now()

        setpoint_pub.publish(pose)
        rate.sleep()

        if current_state.mode == "OFFBOARD" and current_state.armed:
            rospy.loginfo(f"{uav_ns} is in OFFBOARD mode and armed")
            break

    while not rospy.is_shutdown():
        setpoint_pub.publish(pose)
        rate.sleep()

def main():
    rospy.init_node('multi_uav_takeoff_hover', anonymous=True)
    rospy.loginfo("multi_uav_takeoff_hover node started")

    uav_list = ['uav1', 'uav2', 'uav3', 'uav4', 'uav5', 'uav6']

    condition = threading.Condition()
    ready_count = [0]
    takeoff_time = [None]

    threads = []
    for uav in uav_list:
        rospy.loginfo(f"Starting thread for {uav}")
        t = threading.Thread(target=takeoff_hover, args=(uav, condition, ready_count, takeoff_time))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
