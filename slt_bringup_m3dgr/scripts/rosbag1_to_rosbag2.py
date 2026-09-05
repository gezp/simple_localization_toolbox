#!/usr/bin/env python3
# Copyright 2026 Gezp (https://github.com/gezp).
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""Convert M3DGR ROS1 bag to ROS2 mcap bag.

- livox CustomMsg (/livox/*/lidar) -> PointCloud2 (/livox/*/points),
  fields: x y z intensity ring time — time is absolute per-point timestamp
  in seconds (float64), layout matches slt_common::PointXYZIRT
- pass-through: IMU, wheel /odom, NavSatFix, CompressedImage
- skip: mavros fix (always NO_FIX), gnss_comm raw msgs, CustomMsg originals

Usage:
  python3 rosbag1_to_rosbag2.py <input.bag> <output_dir> [--topics t1 t2 ...]
"""

import argparse
import struct
import sys
import time
from pathlib import Path

import numpy as np

from rosbags.highlevel import AnyReader
from rosbags.rosbag2 import StoragePlugin, Writer as Rosbag2Writer
from rosbags.typesys import Stores, get_typestore

POINT_CLOUD_2 = 'sensor_msgs/msg/PointCloud2'
POINT_STEP = 32  # x y z intensity (f32) + ring (u8) + pad + time (f64 absolute sec)


def ros1_to_ros2_header(msg, ts):
    """Rewrite header.stamp type (ROS1 time -> ROS2 builtin_interfaces/Time)."""
    header = getattr(msg, 'header', None)
    if header is None or not hasattr(header, 'stamp'):
        return
    old = header.stamp
    header.stamp = ts.types['builtin_interfaces/msg/Time'](
        sec=old.secs if hasattr(old, 'secs') else old.sec,
        nanosec=old.nsecs if hasattr(old, 'nsecs') else old.nanosec)


def custom_msg_to_pc2(msg, timestamp, ts):
    """livox CustomMsg -> PointCloud2, drops zero-xyz points.

    time = absolute per-point timestamp in seconds (bag time + offset_time),
    float64 to match slt_common::PointXYZIRT (double time).
    """
    PointField = ts.types['sensor_msgs/msg/PointField']
    fields = [
        PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        PointField(name='ring', offset=16, datatype=PointField.UINT8, count=1),
        PointField(name='time', offset=24, datatype=PointField.FLOAT64, count=1),
    ]
    base_sec = timestamp / 1e9
    buf = bytearray()
    xyz_pack = struct.Struct('<fff').pack
    hdr_pack = struct.Struct('<fB').pack
    time_pack = struct.Struct('<d').pack
    for p in msg.points:
        if abs(p.x) + abs(p.y) + abs(p.z) < 1e-6:
            continue
        buf += xyz_pack(p.x, p.y, p.z)
        buf += hdr_pack(float(p.reflectivity), p.line)  # intensity(12-16) ring(16-17)
        buf += b'\x00' * 7                              # pad 17-24
        buf += time_pack(base_sec + p.offset_time * 1e-9)
    n_points = len(buf) // POINT_STEP
    data = np.empty(len(buf), dtype=np.uint8)
    data[:] = buf
    return fields, data, n_points


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', help='input ROS1 bag file or directory')
    parser.add_argument('output', help='output ROS2 bag directory (mcap)')
    parser.add_argument(
        '--topics', nargs='*', default=None,
        help='extra topics to keep (default keep-all list, see source)')
    args = parser.parse_args()

    default_keep = [
        '/livox/mid360/imu',
        '/livox/mid360/lidar',
        '/livox/avia/imu',
        '/livox/avia/lidar',
        '/camera/imu',
        '/odom',
        '/ublox_driver/receiver_lla',
        '/camera/aligned_depth_to_color/image_raw/compressedDepth',
        '/camera/color/image_raw/compressed',
        '/cv_camera/image_raw/compressed',
    ]
    keep = set(args.topics) if args.topics else set(default_keep)

    src = Path(args.input)
    out = Path(args.output)
    if out.exists() and any(out.iterdir()):
        print(f'output dir not empty: {out}', file=sys.stderr)
        return 1
    with AnyReader([src]) as reader:

        dst_typestore = get_typestore(Stores.ROS2_JAZZY)
        writer = Rosbag2Writer(out, version=Rosbag2Writer.VERSION_LATEST,
                               storage_plugin=StoragePlugin.MCAP)
        with writer:
            conns = {}
            for c in reader.connections:
                if c.topic not in keep:
                    continue
                if c.msgtype.endswith('/CustomMsg'):
                    out_topic = c.topic.replace('/lidar', '/points')
                    conn = writer.add_connection(
                        out_topic, POINT_CLOUD_2, typestore=dst_typestore)
                else:
                    conn = writer.add_connection(
                        c.topic, c.msgtype, typestore=dst_typestore)
                conns[c.topic] = conn

            total = sum(c.msgcount for c in reader.connections if c.topic in keep)
            n = 0
            t_start = time.time()
            for conn, timestamp, rawdata in reader.messages(
                    connections=[c for c in reader.connections if c.topic in keep]):
                if conn.msgtype.endswith('/CustomMsg'):
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    fields, data, n_points = custom_msg_to_pc2(
                        msg, timestamp, dst_typestore)
                    Header = dst_typestore.types['std_msgs/msg/Header']
                    stamp = dst_typestore.types['builtin_interfaces/msg/Time'](
                        sec=timestamp // 10**9, nanosec=timestamp % 10**9)
                    frame_id = conn.topic.strip('/').replace('/', '_')
                    pc2 = dst_typestore.types[POINT_CLOUD_2](
                        header=Header(frame_id=frame_id, stamp=stamp),
                        height=1,
                        width=n_points,
                        fields=fields,
                        is_bigendian=False,
                        point_step=POINT_STEP,
                        row_step=POINT_STEP * n_points,
                        data=data,
                        is_dense=True,
                    )
                    writer.write(conns[conn.topic], timestamp,
                                 dst_typestore.serialize_cdr(pc2, POINT_CLOUD_2))
                else:
                    msg = reader.deserialize(rawdata, conn.msgtype)
                    ros1_to_ros2_header(msg, dst_typestore)
                    data = dst_typestore.serialize_cdr(msg, conn.msgtype)
                    writer.write(conns[conn.topic], timestamp, data)
                n += 1
                if n % 5000 == 0:
                    rate = n / (time.time() - t_start)
                    print(f'{n}/{total} msgs ({rate:.0f} msg/s)', flush=True)
    print(f'done: {n} msgs -> {out}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
