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
                "launch/phase3_control.launch.py",
                "launch/phase5_task.launch.py",
                "launch/phase8_modular_grasp.launch.py",
                "launch/phase8_graspgen_shadow.launch.py",
                "launch/phase8_graspgen_viz.launch.py",
                "launch/phase8_vision_web.launch.py",
                "launch/phase8_web_ui.launch.py",
                "launch/manual_pose_debug.launch.py",
                "launch/phase6_vision.launch.py",
                "launch/phase6_sim_base.launch.py",
                "launch/phase6_sim_gazebo.launch.py",
                "launch/split_vm_app.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/phase1_fake_chain.yaml",
                "config/phase2_hardware.yaml",
                "config/phase3_control.yaml",
                "config/phase5_task.yaml",
                "config/phase5_task_hardware.yaml",
                "config/phase8_modular_grasp.yaml",
                "config/manual_pose_debug.yaml",
                "config/phase6_vision.yaml",
                "config/phase6_sim_base.yaml",
                "config/phase6_sim_gazebo.yaml",
                "config/split_vm_app.yaml",
                "config/split_windows_hardware.yaml",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="whisp",
    maintainer_email="whisp@users.noreply.github.com",
    description="Bringup package for tactile ROS2 migration phases 1-6.2.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_tactile_publisher = tactile_bringup.fake_tactile_publisher:main",
        ],
    },
)
