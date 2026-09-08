from pathlib import Path
import csv
import struct
import numpy as np
import open3d as o3d
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
OUT.mkdir(exist_ok=True)

typestore = get_typestore(Stores.ROS1_NOETIC)


def read_xyz_from_pointcloud2(msg):
    field_offsets = {field.name: field.offset for field in msg.fields}
    if not all(name in field_offsets for name in ("x", "y", "z")):
        return np.empty((0, 3), dtype=np.float64)

    points = []
    for row in range(msg.height):
        row_base = row * msg.row_step
        for col in range(msg.width):
            base = row_base + col * msg.point_step
            x = struct.unpack_from("<f", msg.data, base + field_offsets["x"])[0]
            y = struct.unpack_from("<f", msg.data, base + field_offsets["y"])[0]
            z = struct.unpack_from("<f", msg.data, base + field_offsets["z"])[0]
            if np.isfinite(x) and np.isfinite(y) and np.isfinite(z):
                points.append((x, y, z))

    if not points:
        return np.empty((0, 3), dtype=np.float64)

    return np.asarray(points, dtype=np.float64)


def save_ply(xyz, path):
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(xyz)
    o3d.io.write_point_cloud(str(path), pcd)


def extract_frames(bag_relpath, label, max_frames=20):
    bag_path = ROOT / bag_relpath
    out_dir = OUT / label
    out_dir.mkdir(parents=True, exist_ok=True)

    rows = []
    frame_idx = 0

    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == "/velodyne/points"]

        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            xyz = read_xyz_from_pointcloud2(msg)
            if len(xyz) == 0:
                continue

            ply_path = out_dir / f"frame_{frame_idx:04d}.ply"
            save_ply(xyz, ply_path)

            rows.append([frame_idx, timestamp, str(ply_path), len(xyz)])
            frame_idx += 1

            if frame_idx >= max_frames:
                break

    csv_path = OUT / f"{label}_index.csv"
    with csv_path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["frame", "timestamp", "file", "num_points"])
        writer.writerows(rows)

    print(f"{label}: wrote {frame_idx} frames to {out_dir}")


extract_frames(Path("bags") / "sample-ground.bag", "ground", max_frames=20)
extract_frames(Path("bags") / "sample-aerial.bag", "aerial", max_frames=20)
print("Done")