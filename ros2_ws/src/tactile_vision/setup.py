from setuptools import find_packages, setup


package_name = "tactile_vision"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="whisp",
    maintainer_email="whisp@users.noreply.github.com",
    description="Vision bridge nodes for phase 6 ROS2 migration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "realsense_monitor_node = tactile_vision.realsense_monitor_node:main",
            "realsense_camera_node = tactile_vision.realsense_camera_node:main",
        ],
    },
)
