# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import json
import argparse
import time
import datetime
import os, sys
from os.path import isfile

import open3d as o3d

# from open3d_example import check_folder_structure

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from InitializeConfig import initialize_config

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Reconstruction system")
    parser.add_argument("--config",
                        help="path to the config file",
                        default=None)

    args = parser.parse_args()

    if args.config is not None:
        with open(args.config) as json_file:
            config = json.load(json_file)
            initialize_config(config)
            # check_folder_structure(config['path_dataset'])
    else:
        print("Please input'--config [your config path]' and try again!")
        sys.exit()

    assert config is not None

    print("====================================")
    print("Configuration")
    print("====================================")
    for key, val in config.items():
        print("%40s : %s" % (key, str(val)))

    sys.stdout.flush()
