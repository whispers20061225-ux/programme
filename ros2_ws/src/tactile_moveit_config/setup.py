from setuptools import find_packages, setup


package_name = "tactile_moveit_config"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", ".setup_assistant"]),
        (
            f"share/{package_name}/launch",
            [
                "launch/moveit_gazebo_demo.launch.py",
            ],
        ),
        (
            f"share/{package_name}/config",
            [
                "config/dofbot.srdf",
                "config/kinematics.yaml",
                "config/joint_limits.yaml",
                "config/ompl_planning.yaml",
                "config/pilz_cartesian_limits.yaml",
                "config/moveit_controllers.yaml",
                "config/moveit.rviz",
            ],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="whisp",
    maintainer_email="whisp@users.noreply.github.com",
    description="MoveIt 2 config package for the DOFBOT Gazebo simulation.",
    license="MIT",
    tests_require=["pytest"],
)
