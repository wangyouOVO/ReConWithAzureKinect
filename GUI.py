

import open3d as o3d
import open3d.core as o3c
import open3d.visualization.gui as gui
import open3d.visualization.rendering as rendering
from InitializeConfig import initialize_config
import json
import os, sys
import numpy as np
import threading
import time
from  MeshCreator import MeshCreator
def set_enabled(widget, enable):
    widget.enabled = enable
    for child in widget.get_children():
        child.enabled = enable

def read_rgbd_image(color_file, depth_file, convert_rgb_to_intensity,):
    color = o3d.io.read_image(color_file)
    depth = o3d.io.read_image(depth_file)
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color,
        depth,
        depth_scale=1000,
        depth_trunc=1.0,
        convert_rgb_to_intensity=convert_rgb_to_intensity)
    return rgbd_image

class ReconstructionWindow:

    def __init__(self, config, font_id):
        self.config = config
        self.window = gui.Application.instance.create_window(
            'Reconstruction', 1280, 800)

        w = self.window
        em = w.theme.font_size

        spacing = int(np.round(0.25 * em))
        vspacing = int(np.round(0.5 * em))
        margins = gui.Margins(vspacing)

        # First panel
        self.panel = gui.Vert(spacing, margins)


        ## Items in adjustable props
        self.adjustable_prop_grid = gui.VGrid(2, spacing,
                                              gui.Margins(em, 0, em, 0))

        ### Depth max slider
        max_label = gui.Label('Depth max')
        self.max_slider = gui.Slider(gui.Slider.DOUBLE)
        self.max_slider.set_limits(3.0, 6.0)
        self.max_slider.double_value = 3.0
        self.adjustable_prop_grid.add_child(max_label)
        self.adjustable_prop_grid.add_child(self.max_slider)

        ## Application control
        b = gui.ToggleSwitch('Start/Pause')
        b.set_on_clicked(self._on_switch)

        ## Tabs
        tab_margins = gui.Margins(0, int(np.round(0.5 * em)), 0, 0)
        tabs = gui.TabControl()

        ### Input image tab
        tab1 = gui.Vert(0, tab_margins)
        self.input_color_image = gui.ImageWidget()
        self.input_depth_image = gui.ImageWidget()
        tab1.add_child(self.input_color_image)
        tab1.add_fixed(vspacing)
        tab1.add_child(self.input_depth_image)
        tabs.add_tab('Input images', tab1)


        self.panel.add_child(gui.Label('Reconstruction settings'))
        self.panel.add_child(self.adjustable_prop_grid)
        self.panel.add_child(b)
        self.panel.add_stretch()
        self.panel.add_child(tabs)

        # Scene widget
        self.widget3d = gui.SceneWidget()

        # FPS panel
        self.fps_panel = gui.Vert(spacing, margins)
        self.output_fps = gui.Label('FPS: 0.0')
        self.fps_panel.add_child(self.output_fps)

        # Now add all the complex panels
        w.add_child(self.panel)
        w.add_child(self.widget3d)
        w.add_child(self.fps_panel)

        self.widget3d.scene = rendering.Open3DScene(self.window.renderer)
        self.widget3d.scene.set_background([1, 1, 1, 1])

        w.set_on_layout(self._on_layout)
        w.set_on_close(self._on_close)

        self.is_done = False

        self.is_started = False
        self.is_running = False
        self.is_surface_updated = False

        self.idx = 0
        self.poses = []

        # Start running
        threading.Thread(name='UpdateMain', target=self.update_main).start()

    def _on_layout(self, ctx):
        em = ctx.theme.font_size

        panel_width = 20 * em
        rect = self.window.content_rect

        self.panel.frame = gui.Rect(rect.x, rect.y, panel_width, rect.height)

        x = self.panel.frame.get_right()
        self.widget3d.frame = gui.Rect(x, rect.y,
                                       rect.get_right() - x, rect.height)

        fps_panel_width = 7 * em
        fps_panel_height = 2 * em
        self.fps_panel.frame = gui.Rect(rect.get_right() - fps_panel_width,
                                        rect.y, fps_panel_width,
                                        fps_panel_height)

    # Toggle callback: application's main controller
    def _on_switch(self, is_on):
        if not self.is_started:
            gui.Application.instance.post_to_main_thread(
                self.window, self._on_start)
        self.is_running = not self.is_running

    # On start: point cloud buffer and model initialization.
    def _on_start(self):
        max_points = 10000

        pcd_placeholder = o3d.t.geometry.PointCloud(
            o3c.Tensor(np.zeros((max_points, 3), dtype=np.float32)))
        pcd_placeholder.point.colors = o3c.Tensor(
            np.zeros((max_points, 3), dtype=np.float32))
        mat = rendering.MaterialRecord()
        mat.shader = 'defaultUnlit'
        mat.sRGB_color = True
        self.widget3d.scene.scene.add_geometry('points', pcd_placeholder, mat)

        # self.model = o3d.t.pipelines.slam.Model(
        #     self.voxel_size_slider.double_value, 16,
        #     self.est_block_count_slider.int_value, o3c.Tensor(np.eye(4)),
        #     o3c.Device(self.config.device))
        self.is_started = True
        set_enabled(self.adjustable_prop_grid, True)

    def _on_close(self):
        self.is_done = True

        # if self.is_started:
            # print('Saving model to {}...'.format(config.path_npz))
            # self.model.voxel_grid.save(config.path_npz)
            # print('Finished.')

            # mesh_fname = '.'.join(config.path_npz.split('.')[:-1]) + '.ply'
            # print('Extracting and saving mesh to {}...'.format(mesh_fname))
            # mesh = extract_trianglemesh(self.model.voxel_grid, config,
            #                             mesh_fname)
            # print('Finished.')

            # log_fname = '.'.join(config.path_npz.split('.')[:-1]) + '.log'
            # print('Saving trajectory to {}...'.format(log_fname))
            # save_poses(log_fname, self.poses)
            # print('Finished.')

        return True

    def init_render(self, depth_ref, color_ref):
        self.input_depth_image.update_image(
            depth_ref.colorize_depth(100,
                                     0.1,
                                     self.max_slider.double_value).to_legacy())
        self.input_color_image.update_image(color_ref.to_legacy())

        self.window.set_needs_layout()

        bbox = o3d.geometry.AxisAlignedBoundingBox([-5, -5, -5], [5, 5, 5])
        self.widget3d.setup_camera(60, bbox, [0, 0, 0])
        self.widget3d.look_at([0, 0, 0], [0, -1, -3], [0, -1, 0])

    # def update_render(self, input_depth, input_color, pcd, frustum):
    #     self.input_depth_image.update_image(
    #         input_depth.colorize_depth(
    #             float(self.scale_slider.int_value), config.depth_min,
    #             self.max_slider.double_value).to_legacy())
    #     self.input_color_image.update_image(input_color.to_legacy())



        # self.widget3d.scene.remove_geometry("frustum")
        # mat = rendering.MaterialRecord()
        # mat.shader = "unlitLine"
        # mat.line_width = 5.0
        # self.widget3d.scene.add_geometry("frustum", frustum, mat)

    def update_render(self, input_depth, input_color,mesh):
        self.input_depth_image.update_image(
            input_depth.colorize_depth(100,
                                     0.1,
                                     self.max_slider.double_value).to_legacy())
        self.input_color_image.update_image(input_color.to_legacy())

        if self.is_scene_updated:
            self.widget3d.scene.scene.update_geometry(
                    'mesh', mesh)
            print("ok")

    # Major loop
    def update_main(self):
        dataColorPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset/color/00000.jpg"
        dataDepthPath = os.path.dirname(os.path.abspath(__file__)) + "/recorder_dataset/depth/00000.png"
        depth_ref = o3d.t.io.read_image(dataDepthPath)
        color_ref = o3d.t.io.read_image(dataColorPath)

        gui.Application.instance.post_to_main_thread(
            self.window, lambda: self.init_render(depth_ref, color_ref))


        fps_interval_len = 30
        self.idx = 0
        pcd = None

        AzureKinectConfig = o3d.io.AzureKinectSensorConfig()
        print(AzureKinectConfig)
        tsdf_cubic_size = 0.8
        path_intrinsic = os.path.dirname(os.path.abspath(__file__)) + "/config/instrinsic.json"
        intrinsic = o3d.io.read_pinhole_camera_intrinsic(path_intrinsic)
        volume = o3d.pipelines.integration.ScalableTSDFVolume(
            voxel_length = tsdf_cubic_size / 512.0,
            sdf_trunc = 0.04,
            color_type = o3d.pipelines.integration.TSDFVolumeColorType.RGB8)
        self.sensor = o3d.io.AzureKinectSensor(AzureKinectConfig)
        self.sensor.connect(0)
        
        
        start = time.time()
        while not self.is_done:
            if not self.is_started or not self.is_running:
                time.sleep(0.05)
                continue
            rgbdImage = self.sensor.capture_frame(True)
            print("get RGBD Image %d"% self.idx)
            o3d.io.write_image(dataColorPath, rgbdImage.color)
            o3d.io.write_image(dataDepthPath, rgbdImage.depth)
            depth = o3d.t.io.read_image(dataDepthPath)
            color= o3d.t.io.read_image(dataColorPath)
            if (self.idx % 3 == 0):
                volume.reset()
                rgbd_image =  read_rgbd_image(dataColorPath,dataDepthPath,False)
                pose = np.identity(4)
                volume.integrate(rgbd_image, intrinsic, np.linalg.inv(pose))
                # mesh = volume.extract_triangle_mesh()
                mesh = volume.extract_point_cloud()
                self.is_scene_updated = True
            else:
                self.is_scene_updated = False

            # Output FPS
            if (self.idx % fps_interval_len == 0):
                end = time.time()
                elapsed = end - start
                start = time.time()
                self.output_fps.text = 'FPS: {:.3f}'.format(fps_interval_len /
                                                            elapsed)
            gui.Application.instance.post_to_main_thread(
                self.window, lambda: self.update_render(depth,color,mesh))

            self.idx += 1
            self.is_done = self.is_done 

        time.sleep(0.5)


if __name__ == '__main__':
    defaultConfigPath = os.path.dirname(os.path.abspath(__file__)) + "/config/launchconfig.json"
    with open(defaultConfigPath) as json_file:
        config = json.load(json_file)
        initialize_config(config)
    app = gui.Application.instance
    app.initialize()
    mono = app.add_font(gui.FontDescription(gui.FontDescription.MONOSPACE))
    w = ReconstructionWindow(config, mono)
    app.run()
