"""
preprocessor.py
---------------
Handles geometric pre-processing of raw point clouds:
  * Voxel grid down-sampling – reduces density while preserving shape.
  * Statistical outlier removal – optionally strips noisy points.
 
The :class:`Preprocessor` is intentionally stateless beyond its
construction parameters so instances can be reused across multiple
clouds without side effects.
"""

import open3d as o3d

class Preprocessor:
    """Apply geometric pre-processing operations to a point cloud.
 
    Parameters
    ----------
    voxel_size : float
        Edge length (metres) of each voxel cell used during down-sampling.
        Smaller values keep more points; larger values are more aggressive.
    remove_outliers : bool
        When *True* a statistical outlier removal pass is run **after**
        voxel down-sampling.
    nb_neighbors : int
        Number of neighbours examined by the outlier removal algorithm.
    std_ratio : float
        Standard-deviation ratio threshold for outlier removal.
    """
    def __init__(
            self,
            voxel_size: float = 0.005,
            remove_outliers: bool = False,
            num_neighbours: int = 20,
            std_ratio: float = 2.0,
    ) -> None:
        if voxel_size <= 0:
            raise ValueError("voxel_size must be positive")
        
        self._voxel_size = voxel_size
        self._remove_outliers = remove_outliers
        self._num_neighbours = num_neighbours
        self._std_ratio = std_ratio

    def voxel_downsample(
            self, pcd: o3d.geometry.PointCloud
    ) -> o3d.geometry.PointCloud:
        
        """Return a voxel-downsampled copy of *pcd*.
 
        Parameters
        ----------
        pcd:
            Input point cloud (not mutated).
 
        Returns
        -------
        open3d.geometry.PointCloud
            Down-sampled cloud.  Colour and normal attributes are averaged
            per voxel cell when present.
        """
        
        downsampled = pcd.voxel_down_sample(self._voxel_size)
        return downsampled
    
    def remove_statistical_outliers(
            self, pcd: o3d.geometry.PointCloud
    ) -> o3d.geometry.PointCloud:
        
        clean, ind = pcd.remove_statistical_outlier(
            nb_neighbors=self._num_neighbours,
            std_ratio=self._std_ratio
        )
        return clean
    
    @property
    def voxel_size(self) -> float:
        return self._voxel_size