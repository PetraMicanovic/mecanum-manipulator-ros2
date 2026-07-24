import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mecanum_manipulator_control"

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
    description="Platform and arm control package for the mecanum mobile manipulator",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mecanum_platform_controller = mecanum_manipulator_control.mecanum_platform_controller:main",
            "arm_servo_keyboard = mecanum_manipulator_control.arm_servo_keyboard:main",
            "plan_named_pose = mecanum_manipulator_control.plan_named_pose:main",
            "demo = mecanum_manipulator_control.demo:main"
        ],
    },
)
