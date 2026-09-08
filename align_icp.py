from pathlib import Path
import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent

ground_path = ROOT / "output" / "ground" / "frame_0000.ply"
aerial_path = ROOT / "output" / "aerial" / "frame_0000.ply"
out_path = ROOT / "output" / "merged_icp.ply"

ground = o3d.io.read_point_cloud(str(ground_path))
aerial = o3d.io.read_point_cloud(str(aerial_path))

ground_ds = ground.voxel_down_sample(0.20)
aerial_ds = aerial.voxel_down_sample(0.20)

ground_ds.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)
aerial_ds.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=1.0, max_nn=30)
)

initial = np.eye(4)
initial[2, 3] = -5.0

result = o3d.pipelines.registration.registration_icp(
    aerial_ds,
    ground_ds,
    2.0,
    initial,
    o3d.pipelines.registration.TransformationEstimationPointToPoint(),
)

print("Fitness:", result.fitness)
print("RMSE:", result.inlier_rmse)
print("Transformation:\n", result.transformation)

aerial_aligned = aerial.transform(result.transformation.copy())
merged = ground + aerial
o3d.io.write_point_cloud(str(out_path), merged)

np.savetxt(ROOT / "output" / "icp_transform.csv", result.transformation, delimiter=",")
print(f"Saved {out_path}")