from setuptools import find_packages, setup


package_name = "tactile_bringup"

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
                "launch/phase1_fake_chain.launch.py",
                "launch/phase2_hardware.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/phase1_fake_chain.yaml",
                "config/phase2_hardware.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="whisp",
    maintainer_email="whisp@users.noreply.github.com",
    description="Phase 1 bringup package for tactile ROS2 migration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_tactile_publisher = tactile_bringup.fake_tactile_publisher:main",
        ],
    },
)
