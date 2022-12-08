# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------


import numpy as np
import open3d as o3d

class MeshCreator:
    def __init__(self,config) -> None:
        self.config = config
    
    
    def getIntrinsic(self):
        if self.config["path_intrinsic"]:
            self.intrinsic = o3d.io.read_pinhole_camera_intrinsic(
            self.config["path_intrinsic"])
        else:
            self.intrinsic = o3d.camera.PinholeCameraIntrinsic(
            o3d.camera.PinholeCameraIntrinsicParameters.PrimeSenseDefault)
    
    def getPoseGraph(self,pose_graph):
        self.poseGraph = pose_graph
        
    
    def integrate_rgb_frames_for_fragment(self, RGBDList):
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length=self.config["tsdf_cubic_size"] / 512.0,
            sdf_trunc=0.04,
            color_type=o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        for i in range(len(self.poseGraph.nodes)):
            print("integrate rgbd frame %d (%d of %d)." %
                (i, i + 1, len(self.poseGraph.nodes)))
            rgbd = RGBDList[i]
            pose = self.poseGraph.nodes[i].pose
            volume.integrate(rgbd, self.intrinsic, np.linalg.inv(pose))
        mesh = volume.extract_triangle_mesh()
        return mesh
