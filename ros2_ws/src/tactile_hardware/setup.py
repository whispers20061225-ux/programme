from setuptools import find_packages, setup


package_name = "tactile_hardware"

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
    description="Hardware driver nodes for tactile ROS2 migration phase 2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tactile_sensor_node = tactile_hardware.tactile_sensor_node:main",
            "stm32_bridge_node = tactile_hardware.stm32_bridge_node:main",
            "arm_driver_node = tactile_hardware.arm_driver_node:main",
        ],
    },
)
