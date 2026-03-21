import argparse
import sys
from pathlib import Path

from src.loader import PointCloudLoader
from src.preprocessor import Preprocessor
from src.normal_estimator import NormalEstimator

def main():
    loader = PointCloudLoader()
    pcd = loader.load()
    print(f"Loaded {len(pcd.points)} points")

    preprocessor = Preprocessor(voxel_size=0.05)
    down_pcd = preprocessor.voxel_downsample(pcd)
    print(f"Downsampled to {len(down_pcd.points)} points")

if __name__ == "__main__":
    main()