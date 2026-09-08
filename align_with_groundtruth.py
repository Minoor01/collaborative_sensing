from pathlib import Path
import csv
import numpy as np
import open3d as o3d
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"
typestore = get_typestore(Stores.ROS1_NOETIC)


def pose_to_matrix(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    x, y, z, w = q.x, q.y, q.z, q.w

    R = np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ], dtype=np.float64)

    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = [p.x, p.y, p.z]
    return T


def first_pose_from_bag(bag_relpath):
    bag_path = ROOT / bag_relpath
    with Reader(bag_path) as reader:
        connections = [c for c in reader.connections if c.topic == "/gnss/ground_truth"]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = typestore.deserialize_ros1(rawdata, connection.msgtype)
            return timestamp, pose_to_matrix(msg)
    raise RuntimeError(f"No /gnss/ground_truth found in {bag_path}")


def save_matrix_csv(path, mat):
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for row in mat:
            writer.writerow(row.tolist())


ground_ts, T_world_ground = first_pose_from_bag(Path("bags") / "sample-ground.bag")
aerial_ts, T_world_aerial = first_pose_from_bag(Path("bags") / "sample-aerial.bag")

T_ground_from_aerial_init = np.linalg.inv(T_world_ground) @ T_world_aerial

ground = o3d.io.read_point_cloud(str(OUT / "ground" / "frame_0000.ply"))
aerial = o3d.io.read_point_cloud(str(OUT / "aerial" / "frame_0000.ply"))

ground_ds = ground.voxel_down_sample(0.20)
aerial_ds = aerial.voxel_down_sample(0.20)

ground_ds.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)
aerial_ds.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)

result = o3d.pipelines.registration.registration_icp(
    aerial_ds,
    ground_ds,
    2.0,
    T_ground_from_aerial_init,
    o3d.pipelines.registration.TransformationEstimationPointToPlane(),
)

print("Ground timestamp:", ground_ts)
print("Aerial timestamp:", aerial_ts)
print("Initial transform:")
print(T_ground_from_aerial_init)
print("ICP fitness:", result.fitness)
print("ICP RMSE:", result.inlier_rmse)
print("Refined transform:")
print(result.transformation)

aerial_aligned = o3d.io.read_point_cloud(str(OUT / "aerial" / "frame_0000.ply"))
aerial_aligned.transform(result.transformation)
merged = ground + aerial_aligned

o3d.io.write_point_cloud(str(OUT / "merged_groundtruth_icp.ply"), merged)
save_matrix_csv(OUT / "groundtruth_initial_transform.csv", T_ground_from_aerial_init)
save_matrix_csv(OUT / "groundtruth_refined_transform.csv", result.transformation)

print("Saved:")
print(OUT / "merged_groundtruth_icp.ply")
print(OUT / "groundtruth_initial_transform.csv")
print(OUT / "groundtruth_refined_transform.csv")
