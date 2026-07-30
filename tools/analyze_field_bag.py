#!/usr/bin/env python3
import argparse
import math
from collections import Counter, defaultdict

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def sec(ns):
    return ns / 1e9


def finite(x):
    try:
        return math.isfinite(float(x))
    except Exception:
        return False


def yaw_from_quat(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def fmt_t(rel_s):
    return f"+{rel_s:8.1f}s"


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag")
    args = parser.parse_args()

    storage_options = rosbag2_py.StorageOptions(uri=args.bag, storage_id="mcap")
    converter_options = rosbag2_py.ConverterOptions(
        input_serialization_format="cdr", output_serialization_format="cdr"
    )
    reader = rosbag2_py.SequentialReader()
    reader.open(storage_options, converter_options)

    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    msg_classes = {topic: get_message(type_name) for topic, type_name in topic_types.items()}

    counts = Counter()
    first_ts = {}
    last_ts = {}
    max_gap = defaultdict(float)
    big_gaps = defaultdict(list)
    thresholds = {
        "/imu/data": 0.25,
        "/wheel_odom": 0.75,
        "/gps/status": 2.5,
        "/gps/fix": 2.5,
        "/hardware_bridge/status": 2.5,
        "/hardware_bridge/power": 2.5,
        "/hardware_bridge/emergency": 2.5,
        "/odometry/filtered_map": 1.0,
        "/behavior_tree_node/high_level_status": 2.5,
        "/diagnostics": 2.5,
        "/cmd_vel": 2.5,
        "/cmd_vel_nav": 2.5,
        "/cmd_vel_docking": 2.5,
        "/cmd_vel_emergency": 2.5,
    }

    imu_z_abs_max = 0.0
    imu_z_nonzero = 0
    imu_header_backwards = 0
    imu_header_backwards_at = []
    imu_last_header_ns = None

    wheel_yaw_abs_max = 0.0
    filtered_pose = []

    gps_modes = Counter()
    gps_fix_types = Counter()
    gps_quality_min = None
    gps_quality_max = None
    gps_hacc_min = None
    gps_hacc_max = None
    gps_changes = []
    last_gps_state = None

    power_min_battery = None
    power_max_battery = None
    power_status = Counter()
    low_battery_events = []

    behavior_changes = []
    last_behavior = None

    cmd_summary = defaultdict(lambda: {"count": 0, "moving": 0, "spin": 0, "max_lin": 0.0, "max_ang": 0.0})
    cmd_spin_segments = defaultdict(list)
    active_spin_start = {}

    diag_levels = Counter()
    diag_name_levels = defaultdict(Counter)
    diag_imu_age_max = -1.0
    diag_imu_age_at = None
    diag_imu_age_over = []
    diag_warnings = []

    start_ns = None
    end_ns = None

    while reader.has_next():
        topic, data, ts = reader.read_next()
        if start_ns is None:
            start_ns = ts
        end_ns = ts
        counts[topic] += 1
        first_ts.setdefault(topic, ts)
        if topic in last_ts:
            gap = sec(ts - last_ts[topic])
            if gap > max_gap[topic]:
                max_gap[topic] = gap
            if gap > thresholds.get(topic, 5.0):
                big_gaps[topic].append((sec(last_ts[topic] - start_ns), gap))
        last_ts[topic] = ts

        if topic not in msg_classes:
            continue
        msg = deserialize_message(data, msg_classes[topic])
        rel = sec(ts - start_ns)

        if topic == "/imu/data":
            z = float(msg.angular_velocity.z)
            imu_z_abs_max = max(imu_z_abs_max, abs(z))
            if abs(z) > 1e-4:
                imu_z_nonzero += 1
            hns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
            if imu_last_header_ns is not None and hns < imu_last_header_ns:
                imu_header_backwards += 1
                if len(imu_header_backwards_at) < 40:
                    imu_header_backwards_at.append(rel)
            imu_last_header_ns = hns

        elif topic == "/wheel_odom":
            wheel_yaw_abs_max = max(wheel_yaw_abs_max, abs(float(msg.twist.twist.angular.z)))

        elif topic == "/odometry/filtered_map":
            p = msg.pose.pose.position
            yaw = yaw_from_quat(msg.pose.pose.orientation)
            filtered_pose.append((rel, p.x, p.y, yaw))

        elif topic == "/gps/status":
            state = (
                int(msg.fix_type),
                int(msg.rtk_mode),
                bool(msg.corrections_active),
                bool(msg.differential_corrections),
            )
            gps_modes[int(msg.rtk_mode)] += 1
            gps_fix_types[int(msg.fix_type)] += 1
            if finite(msg.quality_percent):
                gps_quality_min = float(msg.quality_percent) if gps_quality_min is None else min(gps_quality_min, float(msg.quality_percent))
                gps_quality_max = float(msg.quality_percent) if gps_quality_max is None else max(gps_quality_max, float(msg.quality_percent))
            if finite(msg.horizontal_accuracy_m):
                gps_hacc_min = float(msg.horizontal_accuracy_m) if gps_hacc_min is None else min(gps_hacc_min, float(msg.horizontal_accuracy_m))
                gps_hacc_max = float(msg.horizontal_accuracy_m) if gps_hacc_max is None else max(gps_hacc_max, float(msg.horizontal_accuracy_m))
            if state != last_gps_state:
                gps_changes.append((rel, state))
                last_gps_state = state

        elif topic == "/hardware_bridge/power":
            vb = float(msg.v_battery)
            power_min_battery = vb if power_min_battery is None else min(power_min_battery, vb)
            power_max_battery = vb if power_max_battery is None else max(power_max_battery, vb)
            power_status[str(msg.charger_status)] += 1
            if vb < 25.5:
                low_battery_events.append((rel, vb, str(msg.charger_status)))

        elif topic == "/behavior_tree_node/high_level_status":
            state = (
                str(msg.state_name),
                str(msg.sub_state_name),
                int(msg.current_area),
                int(msg.current_path),
                int(msg.current_path_index),
            )
            if state != last_behavior:
                behavior_changes.append((rel, state))
                last_behavior = state

        elif topic.startswith("/cmd_vel"):
            tw = msg.twist if hasattr(msg, "twist") else msg
            lin = abs(float(tw.linear.x))
            ang = abs(float(tw.angular.z))
            s = cmd_summary[topic]
            s["count"] += 1
            s["max_lin"] = max(s["max_lin"], lin)
            s["max_ang"] = max(s["max_ang"], ang)
            if lin > 0.02 or ang > 0.05:
                s["moving"] += 1
            spin = lin < 0.03 and ang > 0.25
            if spin:
                s["spin"] += 1
                active_spin_start.setdefault(topic, rel)
            elif topic in active_spin_start:
                start = active_spin_start.pop(topic)
                if rel - start > 2.0:
                    cmd_spin_segments[topic].append((start, rel - start))

        elif topic == "/diagnostics":
            for st in msg.status:
                level = int.from_bytes(st.level, "little") if isinstance(st.level, bytes) else int(st.level)
                diag_levels[level] += 1
                diag_name_levels[str(st.name)][level] += 1
                if level >= 1:
                    diag_warnings.append((rel, level, str(st.name), str(st.message)))
                if str(st.name) == "IMU":
                    for kv in st.values:
                        if kv.key == "age_sec":
                            try:
                                age = float(kv.value)
                            except ValueError:
                                continue
                            if age > diag_imu_age_max:
                                diag_imu_age_max = age
                                diag_imu_age_at = rel
                            if age > 2.0:
                                diag_imu_age_over.append((rel, age))

    for topic, start in active_spin_start.items():
        if end_ns is not None:
            rel_end = sec(end_ns - start_ns)
            if rel_end - start > 2.0:
                cmd_spin_segments[topic].append((start, rel_end - start))

    print(f"Bag: {args.bag}")
    print(f"Duration: {sec(end_ns - start_ns):.1f}s")
    print()
    print("Key topic continuity")
    for topic in [
        "/imu/data",
        "/wheel_odom",
        "/gps/status",
        "/gps/fix",
        "/hardware_bridge/status",
        "/hardware_bridge/power",
        "/hardware_bridge/emergency",
        "/odometry/filtered_map",
        "/behavior_tree_node/high_level_status",
        "/diagnostics",
    ]:
        if counts[topic]:
            rate = counts[topic] / max(sec(last_ts[topic] - first_ts[topic]), 1e-9)
            print(
                f"  {topic:40s} count={counts[topic]:7d} rate={rate:6.2f} Hz "
                f"max_gap={max_gap[topic]:5.3f}s big_gaps={len(big_gaps[topic])}"
            )
        else:
            print(f"  {topic:40s} count=0")

    print()
    print("IMU")
    print(f"  max |angular_velocity.z|: {imu_z_abs_max:.3f} rad/s")
    print(f"  nonzero z samples: {imu_z_nonzero}/{counts['/imu/data']}")
    print(f"  header timestamp backwards events: {imu_header_backwards}")
    if imu_header_backwards_at:
        print("  first header timestamp backwards events:")
        for rel in imu_header_backwards_at[:20]:
            print(f"    {fmt_t(rel)}")
    print(f"  diagnostics max IMU age: {diag_imu_age_max:.3f}s at {fmt_t(diag_imu_age_at or 0)}")
    print(f"  diagnostics IMU age >2s samples: {len(diag_imu_age_over)}")
    if big_gaps["/imu/data"]:
        print("  IMU gaps > threshold:")
        for rel, gap in big_gaps["/imu/data"][:20]:
            print(f"    after {fmt_t(rel)} gap={gap:.3f}s")

    print()
    print("GPS")
    print(f"  rtk_mode counts: {dict(gps_modes)}")
    print(f"  fix_type counts: {dict(gps_fix_types)}")
    print(f"  quality range: {gps_quality_min}..{gps_quality_max}")
    print(f"  horizontal accuracy range: {gps_hacc_min}..{gps_hacc_max}")
    print("  state changes (fix_type, rtk_mode, corrections_active, differential):")
    for rel, state in gps_changes[:30]:
        print(f"    {fmt_t(rel)} {state}")
    if len(gps_changes) > 30:
        print(f"    ... {len(gps_changes) - 30} more")

    print()
    print("Power")
    print(f"  battery voltage range: {power_min_battery}..{power_max_battery}")
    print(f"  charger status counts: {dict(power_status)}")
    if low_battery_events:
        print(f"  low battery events (<25.5V): {len(low_battery_events)} first={low_battery_events[0]}")

    print()
    print("Behavior changes")
    for i, (rel, state) in enumerate(behavior_changes[:80]):
        end_rel = behavior_changes[i + 1][0] if i + 1 < len(behavior_changes) else sec(end_ns - start_ns)
        print(
            f"  {fmt_t(rel)} dur={end_rel - rel:6.1f}s "
            f"state={state[0]} sub={state[1]!r} area={state[2]} path={state[3]} idx={state[4]}"
        )
    if len(behavior_changes) > 80:
        print(f"  ... {len(behavior_changes) - 80} more")

    print()
    print("Command summary")
    for topic, s in sorted(cmd_summary.items()):
        print(
            f"  {topic:22s} count={s['count']:6d} moving={s['moving']:6d} spin={s['spin']:6d} "
            f"max_lin={s['max_lin']:.3f} max_ang={s['max_ang']:.3f}"
        )
        for start, duration in cmd_spin_segments[topic][:10]:
            print(f"    spin segment {fmt_t(start)} duration={duration:.1f}s")
        if len(cmd_spin_segments[topic]) > 10:
            print(f"    ... {len(cmd_spin_segments[topic]) - 10} more spin segments")
        long_spins = [(start, duration) for start, duration in cmd_spin_segments[topic] if duration >= 5.0]
        if long_spins:
            print("    long spin segments >=5s:")
            for start, duration in long_spins[:30]:
                print(f"      {fmt_t(start)} duration={duration:.1f}s")

    print()
    print("Diagnostics")
    print(f"  level counts: {dict(diag_levels)}")
    interesting = []
    for name, levels in diag_name_levels.items():
        if any(level >= 1 and count for level, count in levels.items()):
            interesting.append((name, dict(levels)))
    for name, levels in sorted(interesting)[:80]:
        print(f"  {name}: {levels}")
    print("  first diagnostic warnings/errors:")
    for rel, level, name, message in diag_warnings[:60]:
        print(f"    {fmt_t(rel)} level={level} {name}: {message}")

    print()
    if filtered_pose:
        xs = [p[1] for p in filtered_pose]
        ys = [p[2] for p in filtered_pose]
        print("Filtered pose")
        print(f"  x range: {min(xs):.3f}..{max(xs):.3f}, y range: {min(ys):.3f}..{max(ys):.3f}")
        print(f"  first: {filtered_pose[0]}")
        print(f"  last:  {filtered_pose[-1]}")
        if big_gaps["/odometry/filtered_map"]:
            print("  filtered_map gaps > threshold:")
            for rel, gap in big_gaps["/odometry/filtered_map"][:20]:
                print(f"    after {fmt_t(rel)} gap={gap:.3f}s")


if __name__ == "__main__":
    main()
