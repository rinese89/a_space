#!/usr/bin/env python3
"""Plot odometry topics from a ROS 2 rosbag2 bag.

Supports:
  - nav_msgs/msg/Odometry
  - px4_msgs/msg/VehicleOdometry

Default behavior: plot all nav_msgs/Odometry topics found in the bag. If none
exist, use all px4_msgs/VehicleOdometry topics.
"""

from __future__ import annotations

import argparse
import csv
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple

import matplotlib.pyplot as plt
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

NAV_ODOM_TYPE = "nav_msgs/msg/Odometry"
PX4_ODOM_TYPE = "px4_msgs/msg/VehicleOdometry"
SUPPORTED_TYPES = {NAV_ODOM_TYPE, PX4_ODOM_TYPE}


@dataclass
class OdomSeries:
    topic: str
    msg_type: str
    t: List[float] = field(default_factory=list)
    x: List[float] = field(default_factory=list)
    y: List[float] = field(default_factory=list)
    z: List[float] = field(default_factory=list)
    vx: List[float] = field(default_factory=list)
    vy: List[float] = field(default_factory=list)
    vz: List[float] = field(default_factory=list)
    roll: List[float] = field(default_factory=list)
    pitch: List[float] = field(default_factory=list)
    yaw: List[float] = field(default_factory=list)
    wx: List[float] = field(default_factory=list)
    wy: List[float] = field(default_factory=list)
    wz: List[float] = field(default_factory=list)

    def append(self, t, position, velocity, rpy, angular_velocity):
        self.t.append(float(t))
        self.x.append(float(position[0]))
        self.y.append(float(position[1]))
        self.z.append(float(position[2]))
        self.vx.append(float(velocity[0]))
        self.vy.append(float(velocity[1]))
        self.vz.append(float(velocity[2]))
        self.roll.append(float(rpy[0]))
        self.pitch.append(float(rpy[1]))
        self.yaw.append(float(rpy[2]))
        self.wx.append(float(angular_velocity[0]))
        self.wy.append(float(angular_velocity[1]))
        self.wz.append(float(angular_velocity[2]))


def quaternion_xyzw_to_rpy(x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def decode_nav_odometry(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    v = msg.twist.twist.linear
    w = msg.twist.twist.angular
    return (
        (p.x, p.y, p.z),
        (v.x, v.y, v.z),
        quaternion_xyzw_to_rpy(q.x, q.y, q.z, q.w),
        (w.x, w.y, w.z),
    )


def decode_px4_vehicle_odometry(msg):
    # PX4 VehicleOdometry quaternion uses Hamilton order [w, x, y, z].
    qw, qx, qy, qz = [float(v) for v in msg.q]
    return (
        tuple(float(v) for v in msg.position[:3]),
        tuple(float(v) for v in msg.velocity[:3]),
        quaternion_xyzw_to_rpy(qx, qy, qz, qw),
        tuple(float(v) for v in msg.angular_velocity[:3]),
    )


def create_reader(bag_path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions(
            input_serialization_format="cdr",
            output_serialization_format="cdr",
        ),
    )
    return reader


def get_topics(reader) -> Dict[str, str]:
    return {info.name: info.type for info in reader.get_all_topics_and_types()}


def choose_topics(available: Dict[str, str], requested: Sequence[str]) -> List[str]:
    if requested:
        missing = [t for t in requested if t not in available]
        if missing:
            raise RuntimeError("Topics not found: " + ", ".join(missing))
        unsupported = [t for t in requested if available[t] not in SUPPORTED_TYPES]
        if unsupported:
            raise RuntimeError("Unsupported odometry topic type: " + ", ".join(unsupported))
        return list(requested)

    nav_topics = sorted(t for t, ty in available.items() if ty == NAV_ODOM_TYPE)
    if nav_topics:
        return nav_topics
    return sorted(t for t, ty in available.items() if ty == PX4_ODOM_TYPE)


def load_odometry(bag_path: Path, requested_topics: Sequence[str]) -> Dict[str, OdomSeries]:
    reader = create_reader(bag_path)
    available = get_topics(reader)
    selected = choose_topics(available, requested_topics)
    if not selected:
        raise RuntimeError("No supported odometry topics found in the bag")

    print("Selected topics:")
    for topic in selected:
        print(f"  {topic} [{available[topic]}]")

    msg_classes = {}
    for topic in selected:
        msg_type = available[topic]
        msg_classes[msg_type] = get_message(msg_type)

    data = {topic: OdomSeries(topic, available[topic]) for topic in selected}
    selected_set = set(selected)
    first_timestamp_ns: Optional[int] = None

    while reader.has_next():
        topic, raw, timestamp_ns = reader.read_next()
        if topic not in selected_set:
            continue

        if first_timestamp_ns is None:
            first_timestamp_ns = timestamp_ns
        t = (timestamp_ns - first_timestamp_ns) * 1.0e-9

        msg_type = available[topic]
        msg = deserialize_message(raw, msg_classes[msg_type])

        if msg_type == NAV_ODOM_TYPE:
            decoded = decode_nav_odometry(msg)
        else:
            decoded = decode_px4_vehicle_odometry(msg)

        data[topic].append(t, *decoded)

    return data


def label(topic: str) -> str:
    parts = [p for p in topic.split("/") if p]
    if parts and parts[-1] == "odom" and len(parts) >= 2:
        return parts[-2]
    if parts:
        return parts[0]
    return topic


def save_figure(fig, output_dir: Path, filename: str, enabled: bool):
    if not enabled:
        return
    output_dir.mkdir(parents=True, exist_ok=True)
    path = output_dir / filename
    fig.savefig(path, dpi=160, bbox_inches="tight")
    print(f"Saved: {path}")


def finish_axis(ax, xlabel: str, ylabel: str):
    ax.set_xlabel(xlabel)
    ax.set_ylabel(ylabel)
    ax.grid(True)
    ax.legend()


def plot_scalar(data, attr: str, title: str, ylabel: str, output_dir: Path, filename: str, save: bool, transform=None):
    fig = plt.figure()
    ax = fig.add_subplot(111)
    for topic, series in data.items():
        values = getattr(series, attr)
        if transform is not None:
            values = [transform(v) for v in values]
        ax.plot(series.t, values, label=label(topic))
    ax.set_title(title)
    finish_axis(ax, "Time [s]", ylabel)
    save_figure(fig, output_dir, filename, save)


def plot_all(data: Dict[str, OdomSeries], output_dir: Path, save: bool):
    for axis in ("x", "y", "z"):
        plot_scalar(data, axis, f"Position {axis.upper()} vs time", f"{axis.upper()} [m]", output_dir, f"position_{axis}.png", save)

    for axis in ("vx", "vy", "vz"):
        plot_scalar(data, axis, f"Linear velocity {axis.upper()} vs time", f"{axis.upper()} [m/s]", output_dir, f"velocity_{axis}.png", save)

    for axis in ("roll", "pitch", "yaw"):
        plot_scalar(data, axis, f"{axis.capitalize()} vs time", f"{axis.capitalize()} [deg]", output_dir, f"attitude_{axis}.png", save, math.degrees)

    fig = plt.figure()
    ax = fig.add_subplot(111)
    for topic, s in data.items():
        ax.plot(s.x, s.y, label=label(topic))
        if s.x:
            ax.scatter([s.x[0]], [s.y[0]], marker="o")
            ax.scatter([s.x[-1]], [s.y[-1]], marker="x")
    ax.set_title("XY trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.axis("equal")
    ax.grid(True)
    ax.legend()
    save_figure(fig, output_dir, "trajectory_xy.png", save)

    fig = plt.figure()
    ax = fig.add_subplot(111, projection="3d")
    for topic, s in data.items():
        ax.plot(s.x, s.y, s.z, label=label(topic))
    ax.set_title("3D trajectory")
    ax.set_xlabel("X [m]")
    ax.set_ylabel("Y [m]")
    ax.set_zlabel("Z [m]")
    ax.legend()
    save_figure(fig, output_dir, "trajectory_3d.png", save)


def export_csv(data: Dict[str, OdomSeries], output_dir: Path):
    output_dir.mkdir(parents=True, exist_ok=True)
    header = [
        "time_s", "x_m", "y_m", "z_m", "vx_mps", "vy_mps", "vz_mps",
        "roll_rad", "pitch_rad", "yaw_rad", "wx_radps", "wy_radps", "wz_radps",
    ]
    for topic, s in data.items():
        filename = topic.strip("/").replace("/", "__") + ".csv"
        path = output_dir / filename
        with path.open("w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(header)
            writer.writerows(zip(s.t, s.x, s.y, s.z, s.vx, s.vy, s.vz, s.roll, s.pitch, s.yaw, s.wx, s.wy, s.wz))
        print(f"Saved: {path}")


def print_summary(data: Dict[str, OdomSeries]):
    print("\nSummary:")
    for topic, s in data.items():
        if not s.t:
            print(f"  {topic}: 0 samples")
            continue
        print(
            f"  {topic}: {len(s.t)} samples | t={s.t[0]:.3f}..{s.t[-1]:.3f} s | "
            f"start=({s.x[0]:.3f}, {s.y[0]:.3f}, {s.z[0]:.3f}) | "
            f"end=({s.x[-1]:.3f}, {s.y[-1]:.3f}, {s.z[-1]:.3f})"
        )


def parse_args():
    parser = argparse.ArgumentParser(description="Plot ROS 2 rosbag odometry")
    parser.add_argument("bag", type=Path, help="Rosbag directory containing metadata.yaml and .db3")
    parser.add_argument("--topic", action="append", default=[], help="Topic to plot; repeat for multiple topics")
    parser.add_argument("--output-dir", type=Path, default=None, help="PNG/CSV output directory")
    parser.add_argument("--save-csv", action="store_true", help="Export one CSV per selected topic")
    parser.add_argument("--no-save", action="store_true", help="Do not save PNG files")
    parser.add_argument("--no-show", action="store_true", help="Do not open matplotlib windows")
    return parser.parse_args()


def main():
    args = parse_args()
    bag = args.bag.expanduser().resolve()
    if not bag.exists():
        print(f"ERROR: bag path does not exist: {bag}", file=sys.stderr)
        raise SystemExit(2)

    output_dir = args.output_dir.expanduser().resolve() if args.output_dir else bag / "plots_odometry"

    try:
        data = load_odometry(bag, args.topic)
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(2)

    print_summary(data)
    if args.save_csv:
        export_csv(data, output_dir)

    plot_all(data, output_dir, save=not args.no_save)

    if args.no_show:
        plt.close("all")
    else:
        plt.show()


if __name__ == "__main__":
    main()
