import os
import yaml


CONFIG_PATH = os.path.join(
    os.path.dirname(__file__),
    "../config/gui.yaml"
)


def load_gui_config():

    if not os.path.exists(CONFIG_PATH):
        return {}


    with open(CONFIG_PATH, "r") as f:
        return yaml.safe_load(f) or {}