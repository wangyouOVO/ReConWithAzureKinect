# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import os
import numpy as np
import open3d as o3d
from RGBDdataLoader import read_rgbd_image

class MeshCreator:
    def __init__(self,config,poseGraph) -> None:
        self.config = config
        path_intrinsic = os.path.dirname(os.path.abspath(__file__)) + "/config/instrinsic.json"
        self.intrinsic = o3d.io.read_pinhole_camera_intrinsic(
        path_intrinsic)
        self.poseGraph = poseGraph     
    
    def integrateRgbdFrames(self, colorFiles, depthFiles):
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.config["tsdf_cubic_size"] / 512.0,
            sdf_trunc=0.04,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        for i in range(len(self.poseGraph.nodes)):
            print("integrate rgbd frame %d (%d of %d)." %
                (i, i + 1, len(self.poseGraph.nodes)))
            rgbd_image = read_rgbd_image(colorFiles[i],depthFiles[i],False,self.config)
            print("=============")
            print(rgbd_image)
            print("=============")
            pose = self.poseGraph.nodes[i].pose
            print(pose)
            volume.integrate(rgbd_image, self.intrinsic, np.linalg.inv(pose))
        mesh = volume.extract_triangle_mesh()
        return mesh

    