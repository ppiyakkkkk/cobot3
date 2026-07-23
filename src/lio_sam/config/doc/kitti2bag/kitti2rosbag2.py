#!/usr/bin/env python3
"""KITTI raw dataset -> ROS2 (rosbag2) converter for LIO-SAM-ROS2.

kitti2bag.py in this same folder is ROS1-only (rospy/rosbag/tf/cv_bridge) and
cannot run under a ROS2-only environment. This script re-implements only the
two topics LIO-SAM actually needs (velodyne points + IMU) using the pure
Python 'rosbags' library, so no ROS1 install is required.

Point/IMU field selection mirrors kitti2bag.py's save_velo_data /
save_imu_data_raw functions exactly (ring computation, ax/ay/az + wx/wy/wz
for the high-rate 'extract' IMU), so behavior matches the well-tested
upstream tool.

Usage:
    pip3 install --user rosbags
    python3 kitti2rosbag2.py \
        --kitti_dir ~/kitti_data --date 2011_09_26 --drive 0084 \
        --output ~/kitti_2011_09_26_drive_0084.bag
"""
import argparse
import math
import os
from datetime import datetime

import numpy as np
from rosbags.rosbag2 import Writer
from rosbags.typesys import Stores, get_typestore

FOV_DOWN = -24.8 / 180.0 * math.pi
FOV = (abs(-24.8) + abs(2.0)) / 180.0 * math.pi


def quaternion_from_rpy(roll, pitch, yaw):
    """Equivalent to tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'sxyz')."""
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def read_timestamps(path):
    stamps = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            dt = datetime.strptime(line[:-3], "%Y-%m-%d %H:%M:%S.%f")
            stamps.append(dt.timestamp())
    return stamps


def ring_for_points(scan):
    """scan: (N,4) float32 x,y,z,intensity -> (N,) uint16 ring, HDL-64E geometry."""
    depth = np.linalg.norm(scan[:, :3], axis=1)
    depth = np.where(depth == 0, 1e-6, depth)
    pitch = np.arcsin(scan[:, 2] / depth)
    proj_y = (pitch + abs(FOV_DOWN)) / FOV * 64.0
    proj_y = np.floor(proj_y)
    proj_y = np.clip(proj_y, 0, 63).astype(np.uint16)
    return proj_y


def write_velodyne(writer, ts, conn, velo_dir, frame_id):
    timestamps = read_timestamps(os.path.join(velo_dir, "timestamps.txt"))
    data_dir = os.path.join(velo_dir, "data")
    filenames = sorted(os.listdir(data_dir))

    PointCloud2 = ts.types["sensor_msgs/msg/PointCloud2"]
    PointField = ts.types["sensor_msgs/msg/PointField"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]

    fields = [
        PointField(name="x", offset=0, datatype=7, count=1),
        PointField(name="y", offset=4, datatype=7, count=1),
        PointField(name="z", offset=8, datatype=7, count=1),
        PointField(name="intensity", offset=12, datatype=7, count=1),
        PointField(name="ring", offset=16, datatype=4, count=1),
    ]
    dtype = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                       ("intensity", "<f4"), ("ring", "<u2")])

    print(f"Exporting {len(filenames)} velodyne scans")
    for stamp, filename in zip(timestamps, filenames):
        scan = np.fromfile(os.path.join(data_dir, filename), dtype=np.float32).reshape(-1, 4)
        ring = ring_for_points(scan)

        points = np.zeros(scan.shape[0], dtype=dtype)
        points["x"], points["y"], points["z"], points["intensity"] = (
            scan[:, 0], scan[:, 1], scan[:, 2], scan[:, 3])
        points["ring"] = ring

        t_ns = int(stamp * 1e9)
        header = Header(
            stamp=Time(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000),
            frame_id=frame_id)
        msg = PointCloud2(
            header=header, height=1, width=points.shape[0], fields=fields,
            is_bigendian=False, point_step=dtype.itemsize,
            row_step=dtype.itemsize * points.shape[0],
            data=np.frombuffer(points.tobytes(), dtype=np.uint8), is_dense=True)
        writer.write(conn, t_ns, ts.serialize_cdr(msg, PointCloud2.__msgtype__))


def write_imu_raw(writer, ts, conn, extract_oxts_dir, frame_id):
    timestamps = read_timestamps(os.path.join(extract_oxts_dir, "timestamps.txt"))

    # Linear fit to smooth out raw IMU clock jitter (matches kitti2bag.py).
    idx = np.arange(len(timestamps), dtype=np.float64)
    z = np.polyfit(idx, np.asarray(timestamps, dtype=np.float64), 1)
    timestamps = (z[0] * idx + z[1]).tolist()

    data_dir = os.path.join(extract_oxts_dir, "data")
    filenames = sorted(os.listdir(data_dir))
    assert len(filenames) == len(timestamps), "oxts data/timestamps count mismatch"

    Imu = ts.types["sensor_msgs/msg/Imu"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    Quaternion = ts.types["geometry_msgs/msg/Quaternion"]
    Vector3 = ts.types["geometry_msgs/msg/Vector3"]
    zeros9 = np.zeros(9, dtype=np.float64)

    print(f"Exporting {len(filenames)} raw IMU samples")
    for stamp, filename in zip(timestamps, filenames):
        with open(os.path.join(data_dir, filename)) as f:
            fields = f.read().split()

        roll, pitch, yaw = float(fields[3]), float(fields[4]), float(fields[5])
        qx, qy, qz, qw = quaternion_from_rpy(roll, pitch, yaw)
        ax, ay, az = float(fields[11]), float(fields[12]), float(fields[13])
        wx, wy, wz = float(fields[17]), float(fields[18]), float(fields[19])

        t_ns = int(stamp * 1e9)
        header = Header(
            stamp=Time(sec=t_ns // 1_000_000_000, nanosec=t_ns % 1_000_000_000),
            frame_id=frame_id)
        msg = Imu(
            header=header,
            orientation=Quaternion(x=qx, y=qy, z=qz, w=qw),
            orientation_covariance=zeros9,
            angular_velocity=Vector3(x=wx, y=wy, z=wz),
            angular_velocity_covariance=zeros9,
            linear_acceleration=Vector3(x=ax, y=ay, z=az),
            linear_acceleration_covariance=zeros9)
        writer.write(conn, t_ns, ts.serialize_cdr(msg, Imu.__msgtype__))


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--kitti_dir", required=True, help="Directory containing the date folder")
    parser.add_argument("--date", required=True, help="e.g. 2011_09_26")
    parser.add_argument("--drive", required=True, help="e.g. 0084")
    parser.add_argument("--output", required=True, help="Output bag path (must not exist)")
    parser.add_argument("--points_topic", default="/points")
    parser.add_argument("--imu_topic", default="/imu/data")
    parser.add_argument("--lidar_frame", default="lidar_link")
    parser.add_argument("--imu_frame", default="imu_link")
    args = parser.parse_args()

    date_dir = os.path.join(os.path.expanduser(args.kitti_dir), args.date)
    sync_dir = os.path.join(date_dir, f"{args.date}_drive_{args.drive}_sync")
    extract_dir = os.path.join(date_dir, f"{args.date}_drive_{args.drive}_extract")
    velo_dir = os.path.join(sync_dir, "velodyne_points")
    oxts_dir = os.path.join(extract_dir, "oxts")

    for p in (velo_dir, oxts_dir):
        if not os.path.isdir(p):
            raise SystemExit(f"Not found: {p}\n(did you download+unzip both the *_sync and *_extract archives?)")

    typestore = get_typestore(Stores.ROS2_HUMBLE)
    output = os.path.expanduser(args.output)
    with Writer(output, version=8) as writer:
        points_conn = writer.add_connection(args.points_topic, typestore.types["sensor_msgs/msg/PointCloud2"].__msgtype__, typestore=typestore)
        imu_conn = writer.add_connection(args.imu_topic, typestore.types["sensor_msgs/msg/Imu"].__msgtype__, typestore=typestore)

        write_velodyne(writer, typestore, points_conn, velo_dir, args.lidar_frame)
        write_imu_raw(writer, typestore, imu_conn, oxts_dir, args.imu_frame)

    print(f"Wrote {output}")


if __name__ == "__main__":
    main()
