from setuptools import find_packages, setup


package_name = "tactile_control"

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
    description="Control-layer ROS2 package for tactile migration phase 3.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "arm_control_node = tactile_control.arm_control_node:main",
            "grasp_profile_node = tactile_control.grasp_profile_node:main",
        ],
    },
)
