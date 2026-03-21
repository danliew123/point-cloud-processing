import open3d as o3d

class Preprocessor:
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