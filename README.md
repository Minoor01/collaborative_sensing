# GrAco Small UAV–UGV Registration Prototype

This repository contains a small proof-of-concept pipeline for reading LiDAR data from the **GrAco** dataset, extracting ROS1 `PointCloud2` frames, converting them to `.ply`, and registering aerial and ground point clouds with Open3D.

## Overview

The goal of this project is to test a lightweight UAV–UGV collaborative sensing workflow on a small subset of the GrAco dataset instead of processing the full bags.

The pipeline does the following:

1. Inspect ROS1 bag topics.
2. Extract `/velodyne/points` from ground and aerial bags.
3. Save the first 20 LiDAR frames from each platform as `.ply`.
4. Run ICP registration on selected frame pairs.
5. Visualize source and merged clouds in Open3D.

## Dataset credit

This work uses the **GrAco** dataset.

**Dataset repository:** <https://github.com/SYSU-RoboticsLab/GrAco>  
**Dataset website:** <https://sites.google.com/view/graco-dataset>

**Dataset contributors/authors:**  
Yilin Zhu, Yang Kong, Yingrui Jie, Shiyou Xu, and Hui Cheng from SYSU RAPID Lab.

If you use this dataset in academic work, please cite the dataset authors and any associated paper listed in the official repository.

## Project structure

```text
graco_small/
├── bags/
│   ├── sample-ground.bag
│   └── sample-aerial.bag
├── output/
│   ├── ground/
│   ├── aerial/
│   ├── ground_index.csv
│   ├── aerial_index.csv
│   ├── merged_icp.ply
│   ├── merged_groundtruth_icp.ply
│   ├── merged_refined_search.ply
│   ├── best_icp_transform.csv
│   ├── groundtruth_initial_transform.csv
│   └── groundtruth_refined_transform.csv
├── inspect_bags.py
├── extract_points.py
├── align_icp.py
├── align_with_groundtruth.py
├── refine_icp_search.py
├── view_sections.py
├── requirements.txt
└── README.md
```

## Environment

This project was tested with:

- Python 3.12.3
- Open3D 0.19.0
- NumPy 1.26.4
- rosbags 0.11.5

## Installation

Create and activate a virtual environment:

### PowerShell

```powershell
py -3.12 -m venv .venv
& .\.venv\Scripts\Activate.ps1
```

Install dependencies:

```powershell
python -m pip install -r requirements.txt
```

## Input dataset

Place the GrAco sample bag files in the `bags/` folder:

```text
bags/sample-ground.bag
bags/sample-aerial.bag
```

The prototype was built around these two topics discovered in the bags:

- `/velodyne/points`
- `/gnss/ground_truth`

## Workflow

### 1. Inspect bag topics

```powershell
python inspect_bags.py
```

This prints the topics available in each ROS1 bag.

### 2. Extract LiDAR frames

```powershell
python extract_points.py
```

This reads `/velodyne/points` from both bags and writes the first 20 point-cloud frames to:

- `output/ground/`
- `output/aerial/`

It also creates:

- `output/ground_index.csv`
- `output/aerial_index.csv`

### 3. Run initial ICP registration

```powershell
python align_icp.py
```

This loads:

- `output/ground/frame_0000.ply`
- `output/aerial/frame_0000.ply`

and performs point-cloud registration with Open3D ICP.

### 4. Try ground-truth-initialized registration

```powershell
python align_with_groundtruth.py
```

This attempts to use `/gnss/ground_truth` odometry as the initial transform before ICP refinement.

### 5. Search around the ICP result

```powershell
python refine_icp_search.py
```

This searches small translation offsets around the first ICP solution to test whether a nearby local minimum improves the registration metric.

### 6. Visualize results

```powershell
python view_sections.py
```

This opens separate Open3D windows for:

- Ground cloud
- Aerial cloud
- Merged cloud

## Method summary

The method is:

1. Read ROS1 bags using `rosbags`.
2. Extract XYZ coordinates from `sensor_msgs/PointCloud2`.
3. Save the extracted points as `.ply`.
4. Use Open3D to downsample and estimate normals.
5. Register aerial to ground clouds with ICP.
6. Evaluate the result using fitness, RMSE, and visual inspection.

The `/gnss/ground_truth` poses were also tested as an initialization source, but for the selected first-frame pairing they did not directly provide a usable local registration seed.

## Notes

- Visual inspection is important. A lower ICP RMSE does not always mean the result is physically better.
- The first ICP alignment may be more realistic than the refined-search result, depending on the geometry.
- This repository is a prototype for small-scale testing, not yet a full multi-frame collaborative mapping pipeline.

## Acknowledgment

Thanks to the authors of the GrAco dataset for making a public multimodal ground–aerial cooperative SLAM dataset available.
