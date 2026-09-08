from pathlib import Path
import copy
import open3d as o3d

ROOT = Path(__file__).resolve().parent

ground = o3d.io.read_point_cloud(str(ROOT / "output" / "ground" / "frame_0000.ply"))
aerial = o3d.io.read_point_cloud(str(ROOT / "output" / "aerial" / "frame_0000.ply"))
merged = o3d.io.read_point_cloud(str(ROOT / "output" / "merged_icp.ply"))

ground_col = copy.deepcopy(ground)
aerial_col = copy.deepcopy(aerial)
merged_col = copy.deepcopy(merged)

ground_col.paint_uniform_color([0.2, 0.8, 0.2])   # green
aerial_col.paint_uniform_color([0.9, 0.2, 0.2])   # red
merged_col.paint_uniform_color([0.7, 0.7, 0.7])   # gray

vis1 = o3d.visualization.Visualizer()
vis1.create_window(window_name="Ground", width=640, height=540, left=0, top=40)
vis1.add_geometry(ground_col)

vis2 = o3d.visualization.Visualizer()
vis2.create_window(window_name="Aerial", width=640, height=540, left=650, top=40)
vis2.add_geometry(aerial_col)

vis3 = o3d.visualization.Visualizer()
vis3.create_window(window_name="Merged", width=640, height=540, left=1300, top=40)
vis3.add_geometry(merged_col)

while True:
    alive1 = vis1.poll_events()
    vis1.update_renderer()
    alive2 = vis2.poll_events()
    vis2.update_renderer()
    alive3 = vis3.poll_events()
    vis3.update_renderer()

    if not (alive1 and alive2 and alive3):
        break

vis1.destroy_window()
vis2.destroy_window()
vis3.destroy_window()