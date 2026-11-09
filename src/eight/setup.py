from glob import glob
import os

from setuptools import find_packages, setup


package_name = "eight"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools", "pymavlink"],
    zip_safe=True,
    maintainer="GRA",
    maintainer_email="thunder@example.com",
    description="Figure-eight reference and tracking assessment for px4ctrl X500 SITL",
    license="MIT",
    entry_points={"console_scripts": [
        "eight_node = eight.eight_node:main",
        "takeoff_node = eight.takeoff_node:main",
        "sitl_gcs = eight.sitl_gcs:main",
    ]},
)
