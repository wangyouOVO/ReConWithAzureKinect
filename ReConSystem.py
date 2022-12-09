# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

from PoseSolver import PoseSolver
from MeshCreator import MeshCreator
from open3d import visualization
import open3d as o3d
from RGBDCollector import RecorderOneRGBDWithCallback
class ReConSystem:
    def __init__(self,config,mode) -> None:
        self.mode = mode
        self.config = config
        self.poseSolver = PoseSolver(config)
    def run(self):
        if self.mode == 1:
            self.recorder = RecorderOneRGBDWithCallback()
            RGBDList = self.recorder.run()
            poseGraph = self.poseSolver.getPosegraph(RGBDList)
            for i in range(len(poseGraph.nodes)):
                print(poseGraph.nodes[i].pose)
            self.meshCreater = MeshCreator(self.config,poseGraph=poseGraph)
            meshModel = self.meshCreater.integrateRgbdFrames(RGBDList)
            ply_name = "/home/wt/Projects/ReConWithAzureKinect/recorder_dataset/fragmentmesh.ply"
            o3d.io.write_triangle_mesh(ply_name,meshModel)
            vis = visualization.Visualizer()
            vis.create_window(width=512, height=512)
            vis.add_geometry(meshModel)
            # while True:
            vis.run()
        else:
            pass