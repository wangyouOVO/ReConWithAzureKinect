# ---------------------------------------------------------
#Author: Wentao , Shanghai University, 2022
# ---------------------------------------------------------

import json
import argparse
import os
from InitializeConfig import initialize_config
from ReConSystem import ReConSystem

def getConfig():
    parser = argparse.ArgumentParser(description="Reconstruction system")
    parser.add_argument("--config",
                        help="path to the config file",
                        default=None)

    args = parser.parse_args()

    if args.config is not None:
        with open(args.config) as json_file:
            config = json.load(json_file)
            initialize_config(config)
    else:
        defaultConfigPath = os.path.dirname(os.path.abspath(__file__)) + "/config/launchconfig.json"
        with open(defaultConfigPath) as json_file:
            config = json.load(json_file)
            initialize_config(config)
    assert config is not None

    print("====================================")
    print("Configuration")
    print("====================================")
    for key, val in config.items():
        print("%40s : %s" % (key, str(val)))
    return config

if __name__ == "__main__":
    config = getConfig()
    if config["device_num"] == 1:
        system = ReConSystem(config=config,mode = 1)
        system.run()
    else:
        system = ReConSystem(config=config,mode = 0)
    
