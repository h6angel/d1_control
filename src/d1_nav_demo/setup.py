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
    description="D1 navigation demo: OpenVINS pose to D1 UserCommand.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "fake_bspline_to_path_node = d1_nav_demo.fake_bspline_to_path_node:main",
            "path_tracker_node = d1_nav_demo.path_tracker_node:main",
            "d1_user_command_bridge_node = d1_nav_demo.d1_user_command_bridge_node:main",
        ],
    },
)
