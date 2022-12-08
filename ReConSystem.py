# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import PoseSolver
class ReConSystem:
    def __init__(self,config) -> None:
        self.config = config
        self.pose_graph
        self.poseSolver = PoseSolver.PoseSolver(self.config)