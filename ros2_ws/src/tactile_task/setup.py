from setuptools import find_packages, setup


package_name = "tactile_task"

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
    description="Task orchestration ROS2 package for phase 5 migration.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "demo_task_node = tactile_task.demo_task_node:main",
            "task_executive_node = tactile_task.task_executive_node:main",
            "search_target_skill_node = tactile_task.search_target_skill_node:main",
        ],
    },
)
