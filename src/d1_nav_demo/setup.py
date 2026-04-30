from setuptools import find_packages, setup
from glob import glob

package_name = "d1_nav_demo"

setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="robot",
    maintainer_email="robot@localhost",
    description="D1 navigation demo: OpenVINS pose to Twist on Ubuntu (D1 maps locally).",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "circle_trajectory_tracker_node = d1_nav_demo.circle_trajectory_tracker_node:main",
            "bspline_trajectory_tracker_node = d1_nav_demo.bspline_trajectory_tracker_node:main",
            "bspline_formula_tracker_node = d1_nav_demo.bspline_formula_tracker_node:main",
        ],
    },
)
