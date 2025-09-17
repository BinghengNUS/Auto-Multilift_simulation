#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import argparse
import os
import sys
import math
import sqlite3
from typing import Optional, Tuple, List

import numpy as np
import cv2

# These libraries need to be available in a ROS 2 environment
from rosidl_runtime_py.utilities import get_message
from rclpy.serialization import deserialize_message
import rosbag2_py


def is_db3_file(path: str) -> bool:
    return os.path.isfile(path) and path.endswith(".db3")


def open_reader(uri: str) -> rosbag2_py.SequentialReader:
    storage_options = rosbag2_py.StorageOptions(uri=uri, storage_id='sqlite3')
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format='cdr',
        output_serialization_format='cdr'
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)
    return reader


def find_topic_type(reader: rosbag2_py.SequentialReader, topic_name: str) -> Optional[str]:
    for t in reader.get_all_topics_and_types():
        if t.name == topic_name:
            return t.type
    return None


def estimate_fps_and_size(
    uri: str,
    topic_name: str,
    msg_type: str,
    max_scan: int = 2000
) -> Tuple[Optional[float], Optional[Tuple[int, int]]]:
    """
    Pre-scan timestamps and first frame size, return (fps, (width, height))
    """
    reader = open_reader(uri)
    # Re-check topic type (because of new reader)
    ttype = find_topic_type(reader, topic_name)
    if ttype is None:
        return None, None
    msg_cls = get_message(ttype)

    timestamps: List[int] = []
    wh: Optional[Tuple[int, int]] = None

    count = 0
    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue
        msg = deserialize_message(data, msg_cls)
        if wh is None:
            wh = get_image_size(msg, ttype)
        timestamps.append(t)  # nanoseconds
        count += 1
        if count >= max_scan:
            break

    fps = None
    if len(timestamps) >= 2:
        dts = np.diff(sorted(timestamps))
        dts = dts[dts > 0]
        if len(dts) > 0:
            median_dt_ns = float(np.median(dts))
            if median_dt_ns > 0:
                fps = 1.0 / (median_dt_ns * 1e-9)

    return fps, wh


def get_image_size(msg, msg_type: str) -> Optional[Tuple[int, int]]:
    if msg_type == "sensor_msgs/msg/Image":
        return int(msg.width), int(msg.height)
    elif msg_type == "sensor_msgs/msg/CompressedImage":
        return None
    else:
        return None


def to_bgr_numpy(msg, msg_type: str) -> np.ndarray:
    """
    Convert Image or CompressedImage to OpenCV BGR ndarray.
    Does not depend on cv_bridge; handles common encodings: bgr8, rgb8, mono8, bgra8, rgba8.
    """
    if msg_type == "sensor_msgs/msg/CompressedImage":
        # msg.format is usually "jpeg" or "png"
        arr = np.frombuffer(msg.data, dtype=np.uint8)
        img = cv2.imdecode(arr, cv2.IMREAD_COLOR)  # BGR
        if img is None:
            raise RuntimeError("Failed to decode CompressedImage.")
        return img

    # Raw Image
    enc = msg.encoding.lower() if hasattr(msg, "encoding") else "bgr8"
    h = int(msg.height)
    w = int(msg.width)
    step = int(msg.step)
    buf = np.frombuffer(msg.data, dtype=np.uint8)

    # Assemble according to encoding
    if enc in ("bgr8", "rgb8"):
        # step may equal w*3, or larger (row stride)
        img = buf.reshape((h, step))[:, : (w * 3)].reshape((h, w, 3))
        if enc == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    if enc in ("bgra8", "rgba8"):
        img = buf.reshape((h, step))[:, : (w * 4)].reshape((h, w, 4))
        if enc == "rgba8":
            img = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
        else:
            img = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
        return img

    if enc in ("mono8",):
        img = buf.reshape((h, step))[:, : w].reshape((h, w))
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        return img

    # For unsupported encodings, attempt to treat as 3-channel (may fail)
    # You can extend other encodings here (e.g., 16UC1 depth image)
    raise ValueError(f"Unsupported image encoding: {msg.encoding}")


def export_video(
    uri: str,
    topic_name: str,
    out_path: str,
    fps_override: Optional[float] = None,
    fourcc_str: str = "mp4v"
):
    # First pass: determine type
    reader = open_reader(uri)
    msg_type = find_topic_type(reader, topic_name)
    if msg_type is None:
        raise RuntimeError(f"Topic '{topic_name}' not found in bag.")

    # Pre-scan fps and size
    est_fps, est_wh = estimate_fps_and_size(uri, topic_name, msg_type)

    if fps_override is not None and fps_override > 0:
        fps = fps_override
    else:
        fps = est_fps if (est_fps is not None and est_fps > 0.1) else 30.0

    print(f"[INFO] Using FPS = {fps:.3f} (estimated: {est_fps:.3f} if available)" if est_fps else f"[INFO] Using FPS = {fps:.3f}")

    # Second pass: actually write video
    reader = open_reader(uri)
    msg_cls = get_message(msg_type)

    # For Image type, width/height can be known beforehand; for CompressedImage, decode first frame to get size
    writer = None
    frame_count = 0

    fourcc = cv2.VideoWriter_fourcc(*fourcc_str)
    out_dir = os.path.dirname(os.path.abspath(out_path))
    if out_dir and not os.path.exists(out_dir):
        os.makedirs(out_dir, exist_ok=True)

    while reader.has_next():
        (topic, data, t) = reader.read_next()
        if topic != topic_name:
            continue

        msg = deserialize_message(data, msg_cls)
        frame = to_bgr_numpy(msg, msg_type)  # np.ndarray HxWx3, BGR

        if writer is None:
            h, w = frame.shape[:2]
            writer = cv2.VideoWriter(out_path, fourcc, fps, (w, h))
            if not writer.isOpened():
                raise RuntimeError(f"Failed to open VideoWriter for '{out_path}' with size {(w,h)}.")

        writer.write(frame)
        frame_count += 1

    if writer is not None:
        writer.release()

    if frame_count == 0:
        raise RuntimeError(f"No frames found on topic '{topic_name}'.")
    print(f"[DONE] Wrote {frame_count} frames to {out_path}")


def guess_bag_uri(input_path: str) -> str:
    """
    Allow passing a bag directory or a .db3 file:
    - If a directory, assume it's a rosbag2 directory;
    - If a file (.db3), use it directly as uri;
    """
    if os.path.isdir(input_path):
        # Directory: use the directory as uri (rosbag2_py will find sqlite3 internally)
        return os.path.abspath(input_path)
    if is_db3_file(input_path):
        return os.path.abspath(input_path)
    raise FileNotFoundError(f"Input '{input_path}' is neither a directory nor a .db3 file")


def main():
    ap = argparse.ArgumentParser(description="Export /rgb (Image or CompressedImage) from ROS2 bag to a video.")
    ap.add_argument("bag", help="Path to rosbag2 directory OR a .db3 file")
    ap.add_argument("--topic", default="/rgb", help="Topic name to export (default: /rgb)")
    ap.add_argument("--out", default="rgb.mp4", help="Output video file path (e.g., rgb.mp4)")
    ap.add_argument("--fps", type=float, default=None, help="Override FPS (if not set, auto-estimate)")
    ap.add_argument("--fourcc", default="mp4v", help="FourCC codec (default: mp4v; try 'XVID' for .avi)")
    args = ap.parse_args()

    uri = guess_bag_uri(args.bag)
    export_video(uri, args.topic, args.out, fps_override=args.fps, fourcc_str=args.fourcc)


if __name__ == "__main__":
    main()
