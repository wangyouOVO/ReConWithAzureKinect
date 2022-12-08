# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import PoseSolver
import MeshCreator
class ReConSystem:
    def __init__(self,config) -> None:
        self.config = config
        self.poseSolver = PoseSolver.PoseSolver(config)
        self.meshCreater = MeshCreator.MeshCreator(config)
        
    