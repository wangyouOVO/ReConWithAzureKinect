# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------


import open3d as o3d
import os,os.path
import open3d as o3d
import os
import shutil
from os import  makedirs
from os.path import exists
import open3d as o3d

def exists(path):
    """Test whether a path exists.  Returns False for broken symbolic links"""
    try:
        os.stat(path)
    except (OSError, ValueError):
        return False
    return True



def make_clean_folder(path_folder):
    if not exists(path_folder):
        makedirs(path_folder)
    else:
        shutil.rmtree(path_folder)
        makedirs(path_folder)

class RecorderOneRGBDWithCallback:
    def __init__(self, device = 0, align_depth_to_color = True):
        # Global flags
        self.flag_exit = False
        self.align_depth_to_color = align_depth_to_color
        self.sensor = o3d.io.AzureKinectSensor(config = o3d.io.AzureKinectSensorConfig())
        self.sensor.connect(device)
        self.idx = 0
        self.RGBDList = []

    def escape_callback(self, vis):
        self.flag_exit = True

    def space_callback(self, vis):
        dataPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset"
        if(self.idx == 0):
            make_clean_folder(dataPath)
        rgbdImage = self.sensor.capture_frame(True)
        self.RGBDList.append(rgbdImage)
        print(rgbdImage)
        color_filename = dataPath + '/color/{0:05d}.jpg'.format(self.idx)
        print('Writing to {}'.format(color_filename))
        o3d.io.write_image(color_filename, rgbdImage.color)
        depth_filename = dataPath + '/depth/{0:05d}.png'.format(self.idx)
        print('Writing to {}'.format(depth_filename))
        o3d.io.write_image(depth_filename, rgbdImage.depth)
        self.idx += 1
        return False

    def run(self):
        glfw_key_escape = 256
        glfw_key_space = 32
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.register_key_callback(glfw_key_escape, self.escape_callback)
        vis.register_key_callback(glfw_key_space, self.space_callback)

        vis.create_window('recorder', 1920, 540)
        print("Recorder initialized. Press [SPACE] to get one Rgbd Image. "
              "Press [ESC] to exit.")

        vis_geometry_added = False
        while not self.flag_exit:
            rgbd = self.sensor.capture_frame(True)
            if rgbd is None:
                continue

            if not vis_geometry_added:
                vis.add_geometry(rgbd)
                vis_geometry_added = True

            vis.update_geometry(rgbd)
            vis.poll_events()
            vis.update_renderer()
        return self.RGBDList