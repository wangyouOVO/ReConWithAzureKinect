# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------


import os, sys
import numpy as np
import open3d as o3d
from opencv_pose_estimation import pose_estimation
from RGBDdataLoader import read_rgbd_image

def register_one_rgbd_pair(s, t, intrinsic,colorFiles,depthFiles,config):
    source_rgbd_image = read_rgbd_image(colorFiles[s],depthFiles[s],True,config)
    target_rgbd_image = read_rgbd_image(colorFiles[t],depthFiles[t],True,config)
    option = o3d.pipelines.odometry.OdometryOption()
    option.depth_diff_max = config["depth_diff_max"]
    success_5pt, odo_init = pose_estimation(source_rgbd_image,
                                                    target_rgbd_image,
                                                    intrinsic, False)
    print(odo_init)
    if success_5pt:
        print(" matching between frame : %d and %d successfully!"  % ( s, t))
        [success, trans, info
        ] = o3d.pipelines.odometry.compute_rgbd_odometry(
            source_rgbd_image, target_rgbd_image, intrinsic, odo_init,
            o3d.pipelines.odometry.RGBDOdometryJacobianFromHybridTerm(),
            option)
        return [success, trans, info]
    else:
        print(" matching between frame : %d and %d failed!,Please try again"  % ( s, t))
        return [False, np.identity(4), np.identity(6)]
    
class PoseSolver:
    def __init__(self,config) -> None:
        self.config = config
        self.startIdx = 0 
        self.pose_graph = o3d.pipelines.registration.PoseGraph()
    
    def getIntrinsic(self):
        path_intrinsic = os.path.dirname(os.path.abspath(__file__)) + "/config/instrinsic.json"
        self.intrinsic = o3d.io.read_pinhole_camera_intrinsic(
        path_intrinsic)
        
    
    def makePosegraph(self):
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        trans_odometry = np.identity(4)
        self.pose_graph.nodes.append(
            o3d.pipelines.registration.PoseGraphNode(trans_odometry))
        for s in range(self.startIdx, self.endIdx):
            for t in range(s + 1, self.endIdx):
                if t == s + 1:
                    print(" matching between frame : %d and %d" % ( s, t))
                    [success, trans,info] = register_one_rgbd_pair(s, t, self.intrinsic, 
                                            self.RGBDList["color"],self.RGBDList["depth"], self.config)
                    if not success:
                        print("register failed")
                        sys.exit()
                    trans_odometry = np.dot(trans, trans_odometry)
                    trans_odometry_inv = np.linalg.inv(trans_odometry)
                    self.pose_graph.nodes.append(
                        o3d.pipelines.registration.PoseGraphNode(
                            trans_odometry_inv))
                    self.pose_graph.edges.append(
                        o3d.pipelines.registration.PoseGraphEdge(s,
                                                                t,
                                                                trans,
                                                                info,
                                                                uncertain=False))
                if s == 0 and t == self.endIdx - 1:
                    print(" matching between frame : %d and %d" % ( s, t))
                    [success, trans,info] = register_one_rgbd_pair(s, t, self.intrinsic, 
                                                            self.RGBDList["color"],self.RGBDList["depth"], self.config)
                    if  success:
                        self.pose_graph.edges.append(
                            o3d.pipelines.registration.PoseGraphEdge(s,
                                                                    t,
                                                                    trans,
                                                                    info,
                                                                    uncertain=True))
        
    def optimizePosegraph(self):
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Debug)
        method = o3d.pipelines.registration.GlobalOptimizationLevenbergMarquardt()
        criteria = o3d.pipelines.registration.GlobalOptimizationConvergenceCriteria(
        )
        option = o3d.pipelines.registration.GlobalOptimizationOption(
            max_correspondence_distance=self.config["depth_diff_max"],
            edge_prune_threshold=0.25,
            preference_loop_closure=self.config["preference_loop_closure"],
            reference_node=0)
        o3d.pipelines.registration.global_optimization(self.pose_graph, method, criteria,
                                                    option)
        pose_graph_optimized_name = os.path.dirname(os.path.abspath(__file__))+"/config/optimized_pose_graph.json"
        o3d.io.write_pose_graph(pose_graph_optimized_name, self.pose_graph)
        o3d.utility.set_verbosity_level(o3d.utility.VerbosityLevel.Error)
        
    
    def getPosegraph(self,colorFiles,depthFiles):
        self.RGBDList = {"color":colorFiles,"depth":depthFiles}
        self.endIdx = len(colorFiles)
        print(self.endIdx)
        self.getIntrinsic()
        self.makePosegraph()
        self.optimizePosegraph()
        return self.pose_graph