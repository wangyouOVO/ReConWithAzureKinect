import open3d as o3d
filename = "/home/wt/Projects/ReConWithAzureKinect/recorder_dataset/fragmentmesh.ply"
mesh = o3d.io.read_triangle_mesh(filename, False)

vis = o3d.visualization.Visualizer()
vis.create_window(width=512, height=512)
vis.add_geometry(mesh)
# while True:
vis.run()