from setuptools import setup

package_name = "process_mocap"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Your Name",
    maintainer_email="you@example.com",
    description="Map OptiTrack rigid bodies to TurtleBot namespaces (name or first-seen).",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "process_mocap = process_mocap.mapper_node:main",
        ],
    },
)
