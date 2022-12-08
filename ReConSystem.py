# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

from PoseSolver import PoseSolver
from MeshCreator import MeshCreator

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
            self.meshCreater = MeshCreator(self.config,poseGraph=poseGraph)
            meshModel = self.meshCreater.integrateRgbdFrames(RGBDList)
            
        else:
            pass