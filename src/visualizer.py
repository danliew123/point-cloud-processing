from pathlib import Path
 
import numpy as np
import open3d as o3d

class Visualizer:
    WIDTH: int = 1280
    HEIGHT: int = 720

    @staticmethod
    def save_render(
            pcd: o3d.geometry.PointCloud,
            output_path: str | Path,
            window_name: str = "Point Cloud",
            show_normals: bool = False,
            point_size: float = 2.0,
    ) -> None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        geometries: list[o3d.geometry.Geometry] = [pcd]
        if show_normals and pcd.has_normals():
            normal_lines = Visualizer._normal_line_set(pcd)
            geometries.append(normal_lines)

        Visualizer._offscreen_render(
            geometries,
            str(output_path),
            window_name=window_name,
            point_size=point_size,
        )

        print(f"Saved -> {output_path}")

    @staticmethod
    def save_clusters(
        clusters: list[o3d.geometry.PointCloud],
        output_path: str | Path,
        window_name: str = "Clusters",
        point_size: float = 2.0,
    ) -> None:
        output_path = Path(output_path)
        output_path.parent.mkdir(parents=True, exist_ok=True)

        Visualizer._offscreen_render(
            clusters,
            str(output_path),
            window_name=window_name,
            point_size=point_size
        )
        print(f"Saved -> {output_path}")

    @staticmethod
    def _offscreen_render(
        geometries: list[o3d.geometry.Geometry],
        output_path: str,
        window_name: str = "Render",
        point_size: float = 2.0,
    ) -> None:
        """Render geometries off-screen and write a PNG to output_path"""
        visual = o3d.visualization.Visualizer()
        visual.create_window(
            window_name=window_name,
            width=Visualizer.WIDTH,
            height=Visualizer.HEIGHT,
            visible=False,
        )

        render_option = visual.get_render_option()
        render_option.point_size = point_size
        render_option.background_color = np.array([0.08, 0.08, 0.12])
        render_option.show_coordinate_frame = False

        for geom in geometries:
            visual.add_geometry(geom)

        visual.poll_events()
        visual.update_renderer()
        visual.reset_view_point(True)

        visual.capture_screen_image(output_path, do_render=True)
        visual.destroy_window()


    @staticmethod
    def _normal_line_set(
        pcd: o3d.geometry.PointCloud,
        scale: float = 0.005,
        max_points: int = 5000,
    ) -> o3d.geometry.LineSet:
        points = np.asarray(pcd.points)
        normals = np.asarray(pcd.normals)

        n = min(len(points), max_points)
        idx = np.random.default_rng(0).choice(len(points), n, replace=False)
        pts = points[idx]
        nrm = normals[idx]

        line_points = np.vstack([pts, pts + nrm * scale])
        lines = [[i, i + n] for i in range(n)]

        ls = o3d.geometry.LineSet(
            points=o3d.utility.Vector3dVector(line_points),
            lines=o3d.utility.Vector2iVector(lines),
        )
        ls.paint_uniform_color([0.8, 0.8, 0.2])
        return ls
 