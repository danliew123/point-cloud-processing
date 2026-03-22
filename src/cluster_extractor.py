import numpy as np
import open3d as o3d

_PALETTE = np.array(
    [
        [0.902, 0.224, 0.224],  # coral-red
        [0.224, 0.573, 0.902],  # sky-blue
        [0.224, 0.784, 0.443],  # mint-green
        [0.980, 0.714, 0.090],  # amber
        [0.678, 0.278, 0.902],  # violet
        [0.902, 0.502, 0.224],  # orange
        [0.278, 0.839, 0.839],  # cyan
        [0.902, 0.224, 0.620],  # magenta-pink
    ],
    dtype=np.float64,
)

class ClusterExtractor:
    def __init__(
            self,
            eps: float = 0.02,
            min_points: int = 10,
            print_progress: bool = False,
    ) -> None:
        if eps <= 0:
            raise ValueError(f"eps must be positive, got {eps}")
        if min_points < 1:
            raise ValueError(f"min_points must be >= 1, got {min_points}")
        
        self._eps = eps
        self._min_points = min_points
        self._print_progress = print_progress

    def extract(
            self, pcd: o3d.geometry.PointCloud
    ) -> list[o3d.geometry.PointCloud]:
        if pcd.is_empty():
            raise ValueError("Cannot cluster empty point cloud")
        
        labels = np.asarray(
            pcd.cluster_dbscan(
                eps=self._eps,
                min_points=self._min_points,
                print_progress=self._print_progress,
            )
        )

        unique_labels = set(labels) - {-1}
        clusters: list[o3d.geometry.PointCloud] = []

        for label in unique_labels:
            mask = labels == label
            cluster_pcd = pcd.select_by_index(np.where(mask)[0])
            color = self._color_for(int(label))
            cluster_pcd.paint_uniform_color(color)
            clusters.append(cluster_pcd)
        
        clusters.sort(key=lambda c: len(c.points), reverse=True)
        return clusters

    def label_array(
            self, 
            pcd: o3d.geometry.PointCloud
            ) -> np.ndarray:
        return np.asarray(
            pcd.cluster_dbscan(
                eps=self.eps,
                min_points=self._min_points,
                print_progress=False,
            )
        )


    @staticmethod
    def _color_for(label: int) -> list[float]:
        if label < len(_PALETTE):
            return _PALETTE[label].tolist()
        random_color = np.random.default_rng(seed=label)
        return random_color.uniform(0.2, 0.9, size=3).tolist()
    
    @property
    def eps(self) -> float:
        return self._eps
    
    @property
    def min_points(self) -> int:
        return self._min_points