import os
from glob import glob
from setuptools import find_packages, setup

package_name = "mobile_platform_sim"

data_files = []
data_files.append(
    ("share/ament_index/resource_index/packages", ["resource/" + package_name])
)
data_files.append(("share/" + package_name, ["package.xml"]))

# Launch files
data_files.append(("share/" + package_name + "/launch", glob("launch/*.py")))
# RViz config files
data_files.append(("share/" + package_name + "/config", glob("config/*.rviz")))
# Resource files (URDF)
data_files.append(("share/" + package_name + "/resource", glob("resource/*.urdf")))
# World files
data_files.append(("share/" + package_name + "/worlds", glob("worlds/*.wbt")))
# Mesh files
data_files.append(
    ("share/" + package_name + "/meshes", glob("meshes/*.stl") + glob("meshes/*.STL"))
)

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=data_files,
    install_requires=["setuptools", "scipy"],
    zip_safe=True,
    maintainer="Petra Micanovic",
    maintainer_email="micanovic.petra@yahoo.com",
    description="Webots simulation of a mecanum-wheeled mobile platform",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "odometry_publisher = mobile_platform_sim.odometry_publisher:main",
        ],
    },
)