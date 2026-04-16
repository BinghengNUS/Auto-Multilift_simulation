#!/usr/bin/env python3
"""
px4_world_tf_static.py
-------------------------------------------------
* One-shot   /drone_i_init_pos  -> STATIC TF world->px4_i_frame
* VehicleOdometry (NED/FRD) -> Pose (ENU/FLU)
* Publish    /simulation/position_drone_i  @ 250 Hz
-------------------------------------------------
Author : Yichao Gao 
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseStamped
from px4_msgs.msg import VehicleOdometry
import tf2_ros
from rclpy.qos import (
    QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy
)
from px4_offboard.get_data import DataLoader

# --------------------------------------------------------------------------- #
# QoS profile identical to PX4 default sensor streams
# --------------------------------------------------------------------------- #
def sensor_qos(depth: int = 1) -> QoSProfile:
    return QoSProfile(
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=depth,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


# --------------------------------------------------------------------------- #
# NED -> ENU (position):  swap x/y, invert z
# --------------------------------------------------------------------------- #
def ned_to_enu(pos_ned):
    return pos_ned[1], pos_ned[0], -pos_ned[2]


# --------------------------------------------------------------------------- #
# FRD-quaternion (NED) -> FLU-quaternion (ENU)
#
#   q_enu = q_N2E  ⊗  q_ned  ⊗  q_FRD2FLUᵀ
#
#   q_N2E      : rotates world frame NED -> ENU
#   q_FRD2FLUᵀ : inverse (transpose) of 180° roll (FRD -> FLU)
# --------------------------------------------------------------------------- #
def quat_ned_frd_to_enu_flu(q_wxyz):
    # helper – quaternion product  (w,x,y,z) × (w,x,y,z)
    def qmul(a, b):
        w1, x1, y1, z1 = a
        w2, x2, y2, z2 = b
        return (
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        )

    # rotation NED -> ENU  (180° about axis (1,1,0)/√2)
    SQRT2_INV = 0.70710678118
    q_n2e = (0.0, SQRT2_INV, SQRT2_INV, 0.0)

    # rotation FRD -> FLU  (180° roll)  – inverse needed -> negate vector part
    q_frd2flu_inv = (0.0, -1.0, 0.0, 0.0)  # same rotation, inverse quaternion

    # combine
    q_tmp = qmul(q_n2e, q_wxyz)           # world-frame change
    q_enu = qmul(q_tmp, q_frd2flu_inv)    # body-frame change

    # reorder to (x,y,z,w) for ROS messages
    w, x, y, z = q_enu
    return x, y, z, w


class PX4WorldTF(Node):
    def __init__(self):
        super().__init__('px4_world_tf_static')

        self.drones = DataLoader().num_drones  # number of drones
        self.namespaces = ['', '/px4_1', '/px4_2', '/px4_3', '/px4_4', '/px4_5', '/px4_6']
        self.publish_rate_hz = 250.0

        self.init_tf   = [None] * self.drones   # first /drone_i_init_pos
        self.static_ok = [False] * self.drones
        self.last_pose = [None] * self.drones   # latest converted pose

        self.static_bc = tf2_ros.StaticTransformBroadcaster(self)
        self.tf_bc     = tf2_ros.TransformBroadcaster(self)

        # publishers
        self.sim_pub = [
            self.create_publisher(PoseStamped,
                                  f'/simulation/position_drone_{i+1}', 10)
            for i in range(self.drones)
        ]

        # subscriptions
        for i, ns in enumerate(self.namespaces):
            self.create_subscription(
                TransformStamped, f'/drone_{i}_init_pos',
                lambda msg, i=i: self._store_init(msg, i), 10)

            self.create_subscription(
                VehicleOdometry, f'{ns}/fmu/out/vehicle_odometry',
                lambda msg, i=i: self._handle_odom(msg, i), sensor_qos())

        # periodic publisher
        self.create_timer(1.0 / self.publish_rate_hz, self._publish_buffer)

    # ------------------------------------------------------------------ #
    # Save one-shot init TF and broadcast as static transform
    # ------------------------------------------------------------------ #
    def _store_init(self, msg: TransformStamped, idx: int):
        if self.init_tf[idx] is not None:
            return                                     # already stored
        self.init_tf[idx] = msg

        static_tf               = TransformStamped()
        static_tf.header.stamp  = self.get_clock().now().to_msg()
        static_tf.header.frame_id   = 'world'
        static_tf.child_frame_id    = f'px4_{idx}_frame'
        static_tf.transform         = msg.transform   # direct copy ENU
        self.static_bc.sendTransform(static_tf)

        self.static_ok[idx] = True
        self.get_logger().info(f'Static TF broadcast for drone_{idx}')

    # ------------------------------------------------------------------ #
    # Convert VehicleOdometry NED/FRD -> Pose ENU/FLU
    # ------------------------------------------------------------------ #
    def _handle_odom(self, msg: VehicleOdometry, idx: int):
        if not self.static_ok[idx]:
            self.get_logger().warn(f'No init pose for drone_{idx}')
            return

        # --- position ---
        px, py, pz = ned_to_enu(msg.position)
        init_tr    = self.init_tf[idx].transform.translation
        world_pos  = (px + init_tr.x,
                      py + init_tr.y,
                      pz + init_tr.z)

        # --- attitude ---
        qx, qy, qz, qw = quat_ned_frd_to_enu_flu(msg.q)

        pose                    = PoseStamped()
        pose.header.stamp       = self.get_clock().now().to_msg()
        pose.header.frame_id    = 'world'
        pose.pose.position.x    = float(world_pos[0])
        pose.pose.position.y    = float(world_pos[1])
        pose.pose.position.z    = float(world_pos[2])
        pose.pose.orientation.x = float(qx)
        pose.pose.orientation.y = float(qy)
        pose.pose.orientation.z = float(qz)
        pose.pose.orientation.w = float(qw)

        self.last_pose[idx] = pose

    # ------------------------------------------------------------------ #
    # Publish buffered poses + dynamic TF for RViz
    # ------------------------------------------------------------------ #
    def _publish_buffer(self):
        now = self.get_clock().now().to_msg()
        for i, pose in enumerate(self.last_pose):
            if pose is None:
                continue
            pose.header.stamp = now
            self.sim_pub[i].publish(pose)

            tf_msg                    = TransformStamped()
            tf_msg.header             = pose.header
            tf_msg.child_frame_id     = f'drone_{i}_world'
            tf_msg.transform.translation.x = pose.pose.position.x
            tf_msg.transform.translation.y = pose.pose.position.y
            tf_msg.transform.translation.z = pose.pose.position.z
            tf_msg.transform.rotation      = pose.pose.orientation
            self.tf_bc.sendTransform(tf_msg)


def main():
    rclpy.init()
    node = PX4WorldTF()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
