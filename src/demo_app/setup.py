import os
from glob import glob
from setuptools import find_packages, setup

package_name = "demo_app"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Petra Micanovic",
    maintainer_email="micanovic.petra@yahoo.com",
    description="Demo application for the mecanum-wheeled mobile platform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mecanum_platform_controller = demo_app.mecanum_platform_controller:main",
            "arm_servo_keyboard = demo_app.arm_servo_keyboard:main",
            "plan_named_pose = demo_app.plan_named_pose:main"
        ],
    },
)
