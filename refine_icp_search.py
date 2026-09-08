from pathlib import Path
import csv
import itertools
import numpy as np
import open3d as o3d

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "output"

ground = o3d.io.read_point_cloud(str(OUT / "ground" / "frame_0000.ply"))
aerial = o3d.io.read_point_cloud(str(OUT / "aerial" / "frame_0000.ply"))

ground_ds = ground.voxel_down_sample(0.15)
aerial_ds = aerial.voxel_down_sample(0.15)

ground_ds.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=30)
)
aerial_ds.estimate_normals(
    search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.8, max_nn=30)
)

base = np.array([
    [ 0.99740076, -0.04949196, -0.05236664,  2.03632755],
    [ 0.05009382,  0.99869199,  0.01024288, -0.41845790],
    [ 0.05179120, -0.01283951,  0.99857539,  0.10936018],
    [ 0.0,         0.0,         0.0,         1.0       ],
], dtype=np.float64)

best = None
rows = []

for dx, dy, dz in itertools.product(
    [-1.0, -0.5, 0.0, 0.5, 1.0],
    [-1.0, -0.5, 0.0, 0.5, 1.0],
    [-0.5, 0.0, 0.5],
):
    init = base.copy()
    init[0, 3] += dx
    init[1, 3] += dy
    init[2, 3] += dz

    result = o3d.pipelines.registration.registration_icp(
        aerial_ds,
        ground_ds,
        1.5,
        init,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(),
    )

    rows.append([dx, dy, dz, result.fitness, result.inlier_rmse])

    score = (result.fitness, -result.inlier_rmse)
    if best is None or score > best[0]:
        best = (score, dx, dy, dz, result)

best_result = best[4]

print("Best offset:", best[1], best[2], best[3])
print("Best fitness:", best_result.fitness)
print("Best RMSE:", best_result.inlier_rmse)
print("Best transform:")
print(best_result.transformation)

aerial_aligned = o3d.io.read_point_cloud(str(OUT / "aerial" / "frame_0000.ply"))
aerial_aligned.transform(best_result.transformation)
merged = ground + aerial_aligned

o3d.io.write_point_cloud(str(OUT / "merged_refined_search.ply"), merged)

with open(OUT / "icp_search_results.csv", "w", newline="", encoding="utf-8") as f:
    writer = csv.writer(f)
    writer.writerow(["dx", "dy", "dz", "fitness", "rmse"])
    writer.writerows(rows)

np.savetxt(OUT / "best_icp_transform.csv", best_result.transformation, delimiter=",")
print("Saved:")
print(OUT / "merged_refined_search.ply")
print(OUT / "icp_search_results.csv")
print(OUT / "best_icp_transform.csv")
