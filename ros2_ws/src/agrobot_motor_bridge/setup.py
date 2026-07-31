from setuptools import find_packages, setup

package_name = "agrobot_motor_bridge"

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
    maintainer="Amen Allah Benassi",
    maintainer_email="amenallahbenassi@gmail.com",
    description="ROS2 motor bridge from /cmd_vel to ESP32 serial commands.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "motor_bridge = agrobot_motor_bridge.motor_bridge:main",
        ],
    },
)
