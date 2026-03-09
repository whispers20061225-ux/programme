import os
from glob import glob
from pathlib import Path

from setuptools import find_packages, setup


package_name = "tactile_sim"
package_root = Path(__file__).resolve().parent
repo_root = package_root.parents[2]
mesh_files = sorted(
    os.path.relpath(path, package_root)
    for path in (repo_root / "models" / "meshes").glob("*")
    if path.is_file()
)

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/gazebo_arm.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/ros2_controllers.yaml",
            ],
        ),
        (
            f"share/{package_name}/urdf",
            sorted(glob("urdf/*.xacro")),
        ),
        (
            f"share/{package_name}/meshes",
            mesh_files,
        ),
        (
            f"share/{package_name}/worlds",
            sorted(glob("worlds/*.world")),
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="whisp",
    maintainer_email="whisp@users.noreply.github.com",
    description="Simulation baseline and Gazebo bridge nodes for phase 6.2 ROS2 migration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tactile_sim_node = tactile_sim.tactile_sim_node:main",
            "arm_sim_driver_node = tactile_sim.arm_sim_driver_node:main",
            "sim_realsense_adapter_node = tactile_sim.sim_realsense_adapter_node:main",
        ],
    },
)
