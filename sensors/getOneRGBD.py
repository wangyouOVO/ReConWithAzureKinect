
import argparse
import datetime
import open3d as o3d


class RecorderOneRGBDWithCallback:

    def __init__(self, config, device, filename, align_depth_to_color):
        # Global flags
        self.flag_exit = False
        self.filename = filename
        self.align_depth_to_color = align_depth_to_color
        self.sensor = o3d.io.AzureKinectSensor(config)
        self.sensor.connect(device)
        self.idx = 0

    def escape_callback(self, vis):
        self.flag_exit = True

    def space_callback(self, vis):
        rgbdImage = self.sensor.capture_frame(True)
        print(rgbdImage)
        color_filename = '/home/wt/Projects/ReConWithAzureKinect/recorder_dataset/color/{0:05d}.jpg'.format(
                        self.idx)
        print('Writing to {}'.format(color_filename))
        o3d.io.write_image(color_filename, rgbdImage.color)

        depth_filename = '/home/wt/Projects/ReConWithAzureKinect/recorder_dataset/depth/{0:05d}.png'.format(
            self.idx)
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



if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='Azure kinect mkv recorder.')
    parser.add_argument('--config', type=str, help='input json kinect config')
    parser.add_argument('--output',
                        type=str,
                        help='output path to store color/ and depth/ images')
    parser.add_argument('--list',
                        action='store_true',
                        help='list available azure kinect sensors')
    parser.add_argument('--device',
                        type=int,
                        default=0,
                        help='input kinect device id')
    parser.add_argument('-a',
                        '--align_depth_to_color',
                        action='store_true',
                        help='enable align depth image to color')
    args = parser.parse_args()

    if args.list:
        o3d.io.AzureKinectSensor.list_devices()
        exit()

    if args.config is not None:
        config = o3d.io.read_azure_kinect_sensor_config(args.config)
    else:
        config = o3d.io.AzureKinectSensorConfig()

    if args.output is not None:
        filename = args.output
    else:
        filename = datetime.datetime.now().timestamp()
    print('Prepare writing to {}'.format(filename))

    device = args.device
    if device < 0 or device > 255:
        print('Unsupported device id, fall back to 0')
        device = 0

    print(args.align_depth_to_color)
    r = RecorderOneRGBDWithCallback(config, device,filename,
                                    True)
    r.run()


