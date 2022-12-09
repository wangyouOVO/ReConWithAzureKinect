# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------


import numpy as np
import open3d as o3d
# import sys

class MeshCreator:
    def __init__(self,config,poseGraph) -> None:
        self.config = config
        if self.config["path_intrinsic"]:
            self.intrinsic = o3d.io.read_pinhole_camera_intrinsic(
            self.config["path_intrinsic"])
        else:
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
        self.poseGraph = poseGraph     
    
    def integrateRgbdFrames(self, RGBDList):
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.config["tsdf_cubic_size"] / 512.0,
            sdf_trunc=0.04,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        for i in range(len(self.poseGraph.nodes)):
            print("integrate rgbd frame %d (%d of %d)." %
                (i, i + 1, len(self.poseGraph.nodes)))
            # rgbd = RGBDList[i]
            rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
                RGBDList[i].color,
                RGBDList[i].depth,
                depth_scale=self.config["depth_scale"],
                depth_trunc=self.config["depth_max"],
                convert_rgb_to_intensity=False)
            print("=============")
            print(rgbd_image)
            # print("--------------")
            # print(rgbd_image)
            print("=============")

            pose = self.poseGraph.nodes[i].pose
            print(pose)
            volume.integrate(rgbd_image, self.intrinsic, np.linalg.inv(pose))
        # sys.exit()
        mesh = volume.extract_triangle_mesh()
        # mesh.compute_vertex_normals()
        return mesh

    