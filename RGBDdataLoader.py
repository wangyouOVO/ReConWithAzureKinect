
import open3d as o3d
import os
from os import listdir
from os.path import isfile, join, splitext
import re
import open3d as o3d

def sorted_alphanum(file_list_ordered):
    convert = lambda text: int(text) if text.isdigit() else text
    alphanum_key = lambda key: [convert(c) for c in re.split('([0-9]+)', key)]
    return sorted(file_list_ordered, key=alphanum_key)

def get_file_list(path, extension=None):
    if extension is None:
        file_list = [path + f for f in listdir(path) if isfile(join(path, f))]
    else:
        file_list = [
            path + f
            for f in listdir(path)
            if isfile(join(path, f)) and splitext(f)[1] == extension
        ]
    file_list = sorted_alphanum(file_list)
    return file_list

def read_rgbd_image(color_file, depth_file, convert_rgb_to_intensity, config):
    color = o3d.io.read_image(color_file)
    depth = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=config["depth_scale"],
        depth_trunc=config["depth_max"],
        convert_rgb_to_intensity=convert_rgb_to_intensity)
    return rgbd_image


def get_rgbd_file_lists():
    dataColorPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset/color/"
    dataDepthPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset/depth/"
    color_files = get_file_list(dataColorPath, ".jpg") + \
            get_file_list(dataColorPath, ".png")
    depth_files = get_file_list(dataDepthPath, ".png")
    return color_files, depth_files