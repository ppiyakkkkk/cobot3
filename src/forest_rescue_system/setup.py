from glob import glob
from setuptools import find_packages, setup


package_name = "forest_rescue_system"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
        (f"share/{package_name}/config", glob("config/*.rviz")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rokey",
    maintainer_email="rokey@example.com",
    description="산림 조난자 탐지 드론의 통합 ROS 2 기본 시스템",
    license="BSD-3-Clause",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "human_detector = forest_rescue_system.human_detector_node:main",
            "victim_localizer = forest_rescue_system.victim_localizer_node:main",
            "mission_manager = forest_rescue_system.mission_manager_node:main",
            "drone_controller = forest_rescue_system.drone_controller_node:main",
            "sensor_tf = forest_rescue_system.sensor_tf_node:main",
            "obstacle_monitor = forest_rescue_system.obstacle_monitor_node:main",
            "rviz_visualization = forest_rescue_system.rviz_visualization_node:main",
        ],
    },
)
