import numpy as np
import open3d as o3d
import pytest
 
from src.loader import PointCloudLoader
from src.preprocessor import Preprocessor
from src.normal_estimator import NormalEstimator
from src.cluster_extractor import ClusterExtractor


@pytest.fixture(scope="module")
def eagle_pcd() -> o3d.geometry.PointCloud:
    """Load the Eagle point cloud once for the whole test module."""
    loader = PointCloudLoader()
    return loader.load()
 
 
@pytest.fixture(scope="module")
def downsampled_pcd(eagle_pcd: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    """Return a downsampled cloud shared across tests that need it."""
    pre = Preprocessor(voxel_size=0.005)
    return pre.voxel_downsample(eagle_pcd)


class TestPointCloudLoader:
    def test_load_returns_nonempty_cloud(self, eagle_pcd: o3d.geometry.PointCloud):
        """Loaded cloud must contain at least one point."""
        assert len(eagle_pcd.points) > 0


class TestPreprocessor:
    def test_voxel_downsample_reduces_point_count(
        self,
        eagle_pcd: o3d.geometry.PointCloud,
    ):
        """
        *** Required test 1 ***
        Voxel downsampling must produce strictly fewer points than the
        original cloud.
        """
        pre = Preprocessor(voxel_size=0.005)
        downsampled = pre.voxel_downsample(eagle_pcd)
        assert len(downsampled.points) < len(eagle_pcd.points), (
            "Expected downsampled cloud to have fewer points than the original, "
            f"but got {len(downsampled.points)} >= {len(eagle_pcd.points)}."
        )

    def test_invalid_voxel_size_raises(self):
        """Non-positive voxel_size must raise ValueError at construction."""
        with pytest.raises(ValueError):
            Preprocessor(voxel_size=0.0)
        with pytest.raises(ValueError):
            Preprocessor(voxel_size=-1.0)

class TestNormalEstimator:
    def test_estimate_populates_normals(
        self, downsampled_pcd: o3d.geometry.PointCloud
    ):
        """Estimated cloud must have a normal vector for every point."""
        est = NormalEstimator(radius=0.02, max_nn=30)
        result = est.estimate(downsampled_pcd)
        assert result.has_normals()
        assert len(result.normals) == len(result.points)

class TestClusterExtractor:
    def test_clustering_produces_multiple_segments(
        self, downsampled_pcd: o3d.geometry.PointCloud
    ):
        """
        *** Required test 2 ***
        Clustering must return more than one segment on the Eagle cloud.
        """
        ext = ClusterExtractor(eps=0.02, min_points=10)
        clusters = ext.extract(downsampled_pcd)
        assert len(clusters) > 1, (
            f"Expected more than 1 cluster, got {len(clusters)}."
            "Consider relaxing eps or min_points parameters."
        )

    def test_clusters_sorted_largest_first(
        self, downsampled_pcd: o3d.geometry.PointCloud
    ):
        """Clusters must be returned in descending order by point count."""
        ext = ClusterExtractor(eps=0.02, min_points=10)
        clusters = ext.extract(downsampled_pcd)
        sizes = [len(c.points) for c in clusters]
        assert sizes == sorted(sizes, reverse=True)