from glob import glob
from pathlib import Path
from setuptools import setup


package_name = "inspection_mission"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml") + glob("config/*.json")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    entry_points={
        "console_scripts": [
            "mission_node = inspection_mission.mission_node:main",
            "vision_node = inspection_mission.vision_node:main",
            "validate_config = inspection_mission.validation:main",
            "interface_probe = inspection_mission.probe:main",
        ],
    },
)
