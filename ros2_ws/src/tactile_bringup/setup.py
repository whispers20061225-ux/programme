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
                "launch/tactile_grasp_studio.launch.py",
                "launch/programme_system.launch.py",
                "launch/programme_grasp_stack.launch.py",
                "launch/tactile_hardware_only.launch.py",
                "launch/vision_pipeline.launch.py",
                "launch/web_console_stack.launch.py",
                "launch/vision_web_stack.launch.py",
                "launch/grasp_shadow_visualization.launch.py",
                "launch/grasp_backend_visualization.launch.py",
                "launch/phase1_fake_chain.launch.py",
                "launch/phase2_hardware.launch.py",
                "launch/phase3_control.launch.py",
                "launch/phase5_task.launch.py",
                "launch/manual_pose_debug.launch.py",
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
                "config/programme_grasp_stack.yaml",
                "config/tactile_hardware_only.yaml",
                "config/manual_pose_debug.yaml",
                "config/vision_pipeline.yaml",
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
    description="Bringup package for the Programme system, tactile hardware path, and supporting debug stacks.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_tactile_publisher = tactile_bringup.fake_tactile_publisher:main",
        ],
    },
)
