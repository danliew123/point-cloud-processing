"""
loader.py
---------
Responsible for loading point cloud data from Open3D's built-in dataset
utilities.  Encapsulates all I/O concerns so the rest of the pipeline
operates exclusively on ``open3d.geometry.PointCloud`` objects.
"""

import open3d as o3d

class PointCloudLoader:
    """Load point cloud data from Open3D bundled datasets or a file path.
 
    Parameters
    ----------
    file_path : str | None
        Optional path to a local ``.pcd`` / ``.ply`` file.  When *None*
        (default) the Eagle dataset shipped with Open3D is used.
    """
    def __init__(self, file_path: str | None = None) -> None:
        self._file_path = file_path

    def load(self) -> o3d.geometry.PointCloud:
        if self._file_path is not None:
            return self._load_from_file(self._file_path)
        return self._load_eagle()
    
    @staticmethod
    def _load_eagle() -> o3d.geometry.PointCloud:
        dataset = o3d.data.EaglePointCloud()
        pcd = o3d.io.read_point_cloud(dataset.path)
        if pcd.is_empty():
            raise RuntimeError("Loaded Eagle point cloud is empty.")
        return pcd
    
    @staticmethod
    def _load_from_file(path: str) -> o3d.geometry.PointCloud:
        pcd = o3d.io.read_point_cloud(path)
        if pcd.is_empty():
            raise FileNotFoundError(
                f"Could not load a point cloud from '{path}'."
            )
        return pcd