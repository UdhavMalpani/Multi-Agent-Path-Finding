from setuptools import setup, find_packages
import os
from glob import glob

package_name = "mapf_ros"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        # Package index
        ("share/ament_index/resource_index/packages",
         [f"resource/{package_name}"]),
        # package.xml
        (f"share/{package_name}", ["package.xml"]),
        # Launch files
        (f"share/{package_name}/launch",
         glob("launch/*.launch.py")),
        # Worlds
        (f"share/{package_name}/worlds",
         glob("worlds/*.world")),
        # Config
        (f"share/{package_name}/config",
         glob("config/*.yaml")),
        # RViz
        (f"share/{package_name}/rviz",
         glob("rviz/*.rviz")),
        # Models
        *[
            (f"share/{package_name}/" + os.path.dirname(f), [f])
            for f in glob("models/**/*", recursive=True)
            if os.path.isfile(f)
        ],
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="MAPF Developer",
    maintainer_email="user@example.com",
    description="MAPF ROS2 package — PBS + Gazebo 11",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mapf_planner   = mapf_ros.mapf_planner_node:main",
            "agent_controller = mapf_ros.agent_controller_node:main",
            "world_spawner  = mapf_ros.world_spawner_node:main",
        ],
    },
)
