# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import os
import numpy as np
import open3d as o3d
import cv2

def read_rgbd_image(color_file, depth_file, convert_rgb_to_intensity,):
    color = o3d.io.read_image(color_file)
    depth = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=1000,
        depth_trunc=1.0,
        convert_rgb_to_intensity=convert_rgb_to_intensity)
    return rgbd_image

tsdf_cubic_size = 0.8
path_intrinsic = os.path.dirname(os.path.abspath(__file__)) + "/config/instrinsic.json"
intrinsic = o3d.io.read_pinhole_camera_intrinsic(path_intrinsic)
volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length = tsdf_cubic_size / 512.0,
            sdf_trunc = 0.04,
            color_type = o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
isExit = False
AzureKinectConfig = o3d.io.AzureKinectSensorConfig()
sensor = o3d.io.AzureKinectSensor(AzureKinectConfig)
sensor.connect(0)
vis = o3d.visualization.Visualizer()
vis.create_window(width=512, height=512)
mesh = o3d.geometry.TriangleMesh()
to_reset = True
vis.add_geometry(mesh)
dataColorPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset/color/00000.jpg"
dataDepthPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset/depth/00000.png"

while not isExit:
    rgbdImage = sensor.capture_frame(True)
    print("get RGBD Image ")
    o3d.io.write_image(dataColorPath, rgbdImage.color)
    o3d.io.write_image(dataDepthPath, rgbdImage.depth)
    rgbd_image =  read_rgbd_image(dataColorPath,dataDepthPath,False)
    print("=============")
    print(rgbd_image)
    print("=============")
    pose = np.identity(4)
    volume.reset()
    volume.integrate(rgbd_image, intrinsic, np.linalg.inv(pose))
    mesh2 = volume.extract_triangle_mesh()
    # pcd = volume.extract_point_cloud()
    
    o3d.io.write_triangle_mesh("/home/wt/Projects/ReConWithAzureKinect/recorder_dataset/a.ply",mesh)
    # pcd = o3d.io.read_point_cloud("/home/wt/Projects/ReConWithAzureKinect/recorder_dataset/a.ply")
    # pcd = np.asarray(pcd.points).reshape((-1, 3))
    # pointcloud.points = o3d.utility.Vector3dVector(pcd)
    vertices = np.asarray(mesh2.vertices).reshape((-1, 3))
    mesh.vertices = o3d.utility.Vector3dVector(vertices)
    vertex_colors = np.asarray(mesh2.vertex_colors).reshape((-1, 3))
    mesh.vertex_colors = o3d.utility.Vector3dVector(vertex_colors)
    triangles = np.asarray(mesh2.triangles).reshape((-1, 3))
    mesh.triangles = o3d.utility.Vector3iVector(triangles)
    # vis.remove_geometry(mesh)
    vis.update_geometry(mesh)
    # 注意，如果使用的是open3d 0.8.0以后的版本，这句话应该改为下面格式
    # vis.update_geometry(pointcloud)
    if to_reset:
        vis.reset_view_point(True)
        to_reset = False
    vis.poll_events()
    vis.update_renderer()
    cv2.waitKey(200)
    







