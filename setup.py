#!/usr/bin/env python3
from setuptools import setup, find_packages
from glob import glob
import os

package_name = "dwa_planner_ros2"

data_files = [
    # 패키지 마커 + manifest
    (
        os.path.join("share", "ament_index", "resource_index", "packages"),
        [os.path.join("resource", package_name)],
    ),
    (os.path.join("share", package_name), ["package.xml"]),
]

# launch / config 폴더 통째로 설치
for dir_name in ("launch", "config"):
    if os.path.isdir(dir_name):
        data_files.append(
            (
                os.path.join("share", package_name, dir_name),
                glob(os.path.join(dir_name, "*")),
            )
        )

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    data_files=data_files,
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="JONGHAKKIM",
    maintainer_email="dewfresh1@inha.edu",
    description="Full-port of AMSL’s DWA Planner to Python/ROS 2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dwa_planner_node = dwa_planner_ros2.dwa_planner_node:main",
            "fake_sensor_publisher = dwa_planner_ros2.fake_sensor_publisher:main",
            "fake_goal_publisher = dwa_planner_ros2.fake_goal_publisher:main",
        ],
    },
)
