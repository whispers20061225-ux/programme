from pathlib import Path

from setuptools import find_packages, setup


package_name = "tactile_web_bridge"
share_root = Path("share") / package_name


def _frontend_dist_data_files() -> list[tuple[str, list[str]]]:
    dist_root = Path("frontend") / "dist"
    if not dist_root.exists():
        return []

    grouped: dict[str, list[str]] = {}
    for file_path in sorted(path for path in dist_root.rglob("*") if path.is_file()):
        relative_dir = file_path.parent.relative_to(dist_root)
        install_dir = share_root / "frontend" / "dist" / relative_dir
        grouped.setdefault(install_dir.as_posix(), []).append(file_path.as_posix())
    return list(grouped.items())


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        *_frontend_dist_data_files(),
    ],
    install_requires=[
        "setuptools",
        "fastapi>=0.110.0",
        "requests>=2.31.0",
        "uvicorn>=0.29.0",
        "websockets>=12.0",
    ],
    zip_safe=True,
    maintainer="whisp",
    maintainer_email="whisp@users.noreply.github.com",
    description="FastAPI and WebSocket bridge for the Programme UI.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "tactile_web_gateway = tactile_web_bridge.web_gateway:main",
        ],
    },
)
