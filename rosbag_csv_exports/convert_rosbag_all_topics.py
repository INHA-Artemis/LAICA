#!/usr/bin/env python3
import argparse
import csv
import json
import re
import sqlite3
from collections import OrderedDict
from pathlib import Path

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def safe_topic_name(topic):
    name = topic.strip("/") or "root"
    return re.sub(r"[^A-Za-z0-9_.-]+", "_", name) + ".csv"


def primitive(value):
    return value is None or isinstance(value, (bool, int, float, str))


def to_plain(value):
    if primitive(value):
        return value
    if isinstance(value, bytes):
        return list(value)
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    if hasattr(value, "tolist"):
        return value.tolist()
    if hasattr(value, "__slots__"):
        return {slot.lstrip("_"): to_plain(getattr(value, slot)) for slot in value.__slots__}
    return str(value)


def flatten_message(value, prefix="", out=None):
    if out is None:
        out = OrderedDict()

    if primitive(value):
        out[prefix] = value
        return out

    if isinstance(value, bytes):
        out[prefix] = json.dumps(list(value), separators=(",", ":"))
        return out

    if isinstance(value, (list, tuple)) or hasattr(value, "tolist"):
        seq = value.tolist() if hasattr(value, "tolist") else value
        out[prefix] = json.dumps(to_plain(seq), separators=(",", ":"))
        return out

    if hasattr(value, "__slots__"):
        for slot in value.__slots__:
            key = slot.lstrip("_")
            next_prefix = f"{prefix}.{key}" if prefix else key
            flatten_message(getattr(value, slot), next_prefix, out)
        return out

    out[prefix] = str(value)
    return out


def collect_topic_rows(connection, topic_id, message_type):
    cls = get_message(message_type)
    rows = []
    headers = OrderedDict()
    headers["bag_time_ns"] = None
    headers["bag_time_sec"] = None

    query = "select timestamp, data from messages where topic_id = ? order by timestamp"
    for timestamp, data in connection.execute(query, (topic_id,)):
        msg = deserialize_message(data, cls)
        flat = flatten_message(msg)
        row = OrderedDict()
        row["bag_time_ns"] = timestamp
        row["bag_time_sec"] = f"{timestamp / 1_000_000_000:.9f}"
        row.update(flat)
        for key in row:
            headers.setdefault(key, None)
        rows.append(row)

    return list(headers.keys()), rows


def write_csv(path, headers, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=headers, extrasaction="ignore")
        writer.writeheader()
        for row in rows:
            writer.writerow(row)


def main():
    parser = argparse.ArgumentParser(description="Convert every topic in a ROS 2 sqlite bag to CSV.")
    parser.add_argument("bag_db3", type=Path)
    parser.add_argument("output_dir", type=Path)
    args = parser.parse_args()

    args.output_dir.mkdir(parents=True, exist_ok=True)
    connection = sqlite3.connect(str(args.bag_db3))
    topics = connection.execute(
        "select id, name, type from topics order by id"
    ).fetchall()

    manifest = []
    for topic_id, topic_name, message_type in topics:
        output_path = args.output_dir / safe_topic_name(topic_name)
        headers, rows = collect_topic_rows(connection, topic_id, message_type)
        write_csv(output_path, headers, rows)
        manifest.append(
            {
                "topic": topic_name,
                "type": message_type,
                "messages": len(rows),
                "csv": output_path.name,
            }
        )
        print(f"{topic_name}: {len(rows)} messages -> {output_path}")

    with (args.output_dir / "manifest.json").open("w") as handle:
        json.dump(manifest, handle, indent=2)


if __name__ == "__main__":
    main()
