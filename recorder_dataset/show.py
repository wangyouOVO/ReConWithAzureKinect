import open3d as o3d
import os
filename = os.path.dirname(os.path.abspath(__file__)) +"/a.ply"
mesh = o3d.io.read_triangle_mesh(filename, False)
# mesh = o3d.io.read_point_cloud(filename)
vis = o3d.visualization.Visualizer()
vis.create_window(width=512, height=512)
vis.add_geometry(mesh)
# while True:
vis.run()