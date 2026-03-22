"""
Point Cloud Processing Pipeline
================================
Entry point for the 3D point cloud processing pipeline using Open3D.
Loads the Eagle dataset, applies downsampling, estimates normals,
performs Euclidean clustering, and saves rendered outputs.
"""

import argparse
from pathlib import Path

from src.loader import PointCloudLoader
from src.preprocessor import Preprocessor
from src.normal_estimator import NormalEstimator
from src.cluster_extractor import ClusterExtractor
from src.visualizer import Visualizer

def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="3D Point Cloud Processing Pipeline using Open3D"
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.03,
        help="Voxel size for downsampling (default: 0.003)",
    )
    parser.add_argument(
        "--normal-radius",
        type=float,
        default=0.15,
        help="Radius for normal estimation neighbour search (default: 0.015)",
    )
    parser.add_argument(
        "--normal-max-nn",
        type=int,
        default=30,
        help="Max nearest neighbours for normal estimation (default: 30)",
    )
    parser.add_argument(
        "--cluster-eps",
        type=float,
        default=0.08,
        help="DBSCAN epsilon (cluster neighbourhood radius) (default: 0.08)",
    )
    parser.add_argument(
        "--cluster-min-points",
        type=int,
        default=25,
        help="Minimum points to form a cluster (default: 25)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="outputs",
        help="Directory to save rendered images (default: outputs/)",
    )
    return parser.parse_args()

def main():

    args = parse_args()
    output_dir = Path(args.output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)

    # 1. Load
    loader = PointCloudLoader()
    pcd = loader.load()
    pcd.transform([[1, 0, 0, 0],
               [0, -1, 0, 0],
               [0, 0, 1, 0],
               [0, 0, 0, 1]])
    print(f"Loaded {len(pcd.points)} points")

    # 2. Downsample
    preprocessor = Preprocessor(voxel_size=args.voxel_size)
    down_pcd = preprocessor.voxel_downsample(pcd)
    print(f"Downsampled to {len(down_pcd.points)} points")

    Visualizer.save_render(down_pcd, output_dir / "downsampled.png",
                           window_name="Downsampled")

    # 3. Estimate normals
    estimator = NormalEstimator(
        radius=args.normal_radius,
        max_nn=args.normal_max_nn,
    )
    pcd_with_normals = estimator.estimate(down_pcd)
    print(f"Normals estimated for {len(pcd_with_normals.points)} points")

    Visualizer.save_render(pcd_with_normals, output_dir / "normals.png",
                           window_name="Normals", show_normals=True)

    # 4. Cluster
    extractor = ClusterExtractor(        
        eps=args.cluster_eps,
        min_points=args.cluster_min_points,
    )
    clusters = extractor.extract(pcd_with_normals)
    print(f"Extracted {len(clusters)} clusters")

    Visualizer.save_clusters(clusters, output_dir / "clusters.png",
                            window_name="Clusters")


if __name__ == "__main__":
    main()