import open3d as o3d

class NormalEstimator:
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