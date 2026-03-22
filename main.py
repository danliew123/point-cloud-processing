import argparse
import sys
from pathlib import Path
import open3d as o3d

from src.loader import PointCloudLoader
from src.preprocessor import Preprocessor
from src.normal_estimator import NormalEstimator
from src.cluster_extractor import ClusterExtractor

def main():
    loader = PointCloudLoader()
    pcd = loader.load()
    print(f"Loaded {len(pcd.points)} points")

    preprocessor = Preprocessor(voxel_size=0.05)
    down_pcd = preprocessor.voxel_downsample(pcd)
    print(f"Downsampled to {len(down_pcd.points)} points")

    o3d.io.write_point_cloud("outputs/downsampled.pcd", down_pcd)
    print("Saved outputs/downsampled.pcd")

    estimator = NormalEstimator(radius=0.02, max_nn=30)
    pcd_with_normals = estimator.estimate(down_pcd)
    print(f"Normals estimated for {len(pcd_with_normals.points)} points")

    o3d.io.write_point_cloud("outputs/normals.pcd", pcd_with_normals)
    print("Saved outputs/normals.pcd")

    extractor = ClusterExtractor(eps=0.08, min_points=10)
    clusters = extractor.extract(pcd_with_normals)
    print(f"Extracted {len(clusters)} clusters")

    for i, cluster in enumerate(clusters):
        path = f"outputs/cluster_{i}.pcd"
        o3d.io.write_point_cloud(path, cluster)
        print(f"Saved {path}")

if __name__ == "__main__":
    main()