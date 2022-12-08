# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import math
import os, sys
import numpy as np
import open3d as o3d

pyexample_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.append(pyexample_path)

from open3d_example import *

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from optimize_posegraph import optimize_posegraph_for_fragment

# check opencv python package
with_opencv = initialize_opencv()
if with_opencv:
    from opencv_pose_estimation import pose_estimation

def integrate_rgb_frames_for_fragment(color_files, depth_files, fragment_id,
                                      n_fragments, pose_graph_name, intrinsic,
                                      config):
    pose_graph = o3d.io.read_pose_graph(pose_graph_name)
    volume = o3d.pipelines.integration.ScalableTSDFVolume(
        voxel_length=config["tsdf_cubic_size"] / 512.0,
        sdf_trunc=0.04,
        color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
    for i in range(len(pose_graph.nodes)):
        i_abs = fragment_id * config['n_frames_per_fragment'] + i
        print(
            "Fragment %03d / %03d :: integrate rgbd frame %d (%d of %d)." %
            (fragment_id, n_fragments - 1, i_abs, i + 1, len(pose_graph.nodes)))
        rgbd = read_rgbd_image(color_files[i_abs], depth_files[i_abs], False,
                               config)
        pose = pose_graph.nodes[i].pose
        volume.integrate(rgbd, intrinsic, np.linalg.inv(pose))
    mesh = volume.extract_triangle_mesh()
    mesh.compute_vertex_normals()
    return mesh


def make_pointcloud_for_fragment(path_dataset, color_files, depth_files,
                                 fragment_id, n_fragments, intrinsic, config):
    mesh = integrate_rgb_frames_for_fragment(
        color_files, depth_files, fragment_id, n_fragments,
        join(path_dataset,
             config["template_fragment_posegraph_optimized"] % fragment_id),
        intrinsic, config)
    pcd = o3d.geometry.PointCloud()
    pcd.points = mesh.vertices
    pcd.colors = mesh.vertex_colors
    pcd_name = join(path_dataset,
                    config["template_fragment_pointcloud"] % fragment_id)
    ply_name = join(path_dataset,"fragmentmesh.ply")
    o3d.io.write_point_cloud(pcd_name, pcd, False, True)
    o3d.io.write_triangle_mesh(ply_name,mesh,True)


def process_single_fragment(fragment_id, color_files, depth_files, n_files,
                            n_fragments, config):
    if config["path_intrinsic"]:
        intrinsic = o3d.io.read_pinhole_camera_intrinsic(
            config["path_intrinsic"])
    else:
        intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    sid = fragment_id * config['n_frames_per_fragment']
    eid = min(sid + config['n_frames_per_fragment'], n_files)

    optimize_posegraph_for_fragment(config["path_dataset"], fragment_id, config)
    make_pointcloud_for_fragment(config["path_dataset"], color_files,
                                 depth_files, fragment_id, n_fragments,
                                 intrinsic, config)


def run(config):

    print("making fragments from RGBD sequence.")
    make_clean_folder(join(config["path_dataset"], config["folder_fragment"]))

    [color_files, depth_files] = get_rgbd_file_lists(config["path_dataset"])
    n_files = len(color_files)
    n_fragments = int(
        math.ceil(float(n_files) / config['n_frames_per_fragment']))

    if config["python_multi_threading"] is True:
        from joblib import Parallel, delayed
        import multiprocessing
        import subprocess
        MAX_THREAD = min(multiprocessing.cpu_count(), n_fragments)
        Parallel(n_jobs=MAX_THREAD)(delayed(process_single_fragment)(
            fragment_id, color_files, depth_files, n_files, n_fragments, config)
                                    for fragment_id in range(n_fragments))
    else:
        for fragment_id in range(n_fragments):
            process_single_fragment(fragment_id, color_files, depth_files,
                                    n_files, n_fragments, config)
