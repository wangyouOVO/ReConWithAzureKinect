# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

from PoseSolver import PoseSolver
from MeshCreator import MeshCreator
from open3d import visualization
import open3d as o3d
from RGBDCollector import RecorderOneRGBDWithCallback
from RGBDdataLoader import get_rgbd_file_lists
import os

class ReConSystem:
    def __init__(self,config,mode) -> None:
        self.mode = mode
        self.config = config
        self.poseSolver = PoseSolver(config)
    def run(self):
        if self.mode == 1:
            self.recorder = RecorderOneRGBDWithCallback()
            self.recorder.run()
            color_files, depth_files = get_rgbd_file_lists()
            poseGraph = self.poseSolver.getPosegraph(color_files, depth_files)
            for i in range(len(poseGraph.nodes)):
                print(poseGraph.nodes[i].pose)
            self.meshCreater = MeshCreator(self.config,poseGraph=poseGraph)
            meshModel = self.meshCreater.integrateRgbdFrames(color_files, depth_files)
            ply_name = os.path.dirname(os.path.abspath(__file__)) +"/recorder_dataset/fragmentmesh.ply"
            o3d.io.write_triangle_mesh(ply_name,meshModel)
            vis = visualization.Visualizer()
            vis.create_window(width=512, height=512)
            vis.add_geometry(meshModel)
            # while True:
            vis.run()
        else:
            pass