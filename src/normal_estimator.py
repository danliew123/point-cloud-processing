"""
normal_estimator.py
--------------------
Estimates per-point surface normals using Open3D's KD-tree search.
 
Surface normals are essential for many downstream tasks such as shading,
feature description, and surface reconstruction.  The class supports both
hybrid (radius + k-NN) and pure k-NN search strategies.
"""

import open3d as o3d

class NormalEstimator:
    """Estimate surface normals for a point cloud.
 
    Uses a hybrid KD-tree search (radius **and** max k-NN) so that
    neighbours are bounded both spatially and numerically, which avoids
    degenerate covariance matrices in sparse regions.
 
    Parameters
    ----------
    radius : float
        Sphere radius (metres) within which to search for neighbours.
    max_nn : int
        Upper bound on the number of neighbours used per point.
    orient_towards_camera : bool
        When *True* normals are re-oriented to face the coordinate origin
        (a common convention for single-scan data).
    """
    def __init__(
            self,
            radius: float = 0.02,
            max_nn: int = 30,
            orient_towards_camera: bool = True,
    ) -> None:
        if radius <= 0:
            raise  ValueError(f"radius must be positive, got {radius}")
        if max_nn < 3:
            raise ValueError(f"max_nn must be >= 3, got {max_nn}")
        self._radius = radius
        self._max_nn = max_nn
        self._orient = orient_towards_camera

    def estimate(
            self, pcd: o3d.geometry.PointCloud
    ) -> o3d.geometry.PointCloud:
        
        """Return a copy of *pcd* with surface normals populated.
 
        Parameters
        ----------
        pcd:
            Input point cloud.  Not mutated.
 
        Returns
        -------
        open3d.geometry.PointCloud
            A new cloud with the ``.normals`` attribute filled.
 
        Raises
        ------
        ValueError
            If *pcd* is empty (no points to estimate from).
        """

        if pcd.is_empty():
            raise ValueError("Cannot estimate normals on an empty point cloud,")
        
        result = o3d.geometry.PointCloud(pcd)

        search_param = o3d.geometry.KDTreeSearchParamHybrid(
            radius=self._radius,
            max_nn=self._max_nn
        )
        result.estimate_normals(search_param=search_param)

        if self._orient:
            result.orient_normals_towards_camera_location(
                camera_location=[0.0, 0.0, 0.0]
            )

        return result
    
    @property
    def radius(self) -> float:
        return self._radius
    
    @property
    def max_nn(self) -> int:
        return self._max_nn