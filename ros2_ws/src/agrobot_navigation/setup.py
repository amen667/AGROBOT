from setuptools import setup

package_name = "agrobot_navigation"

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
    maintainer="Amen Allah Benassi",
    maintainer_email="amenallahbenassi@gmail.com",
    description="Custom map-based navigation for AGROBOT prototype",
    license="MIT",
    entry_points={
        "console_scripts": [
            "map_nav = agrobot_navigation.lidar_nav:main",
            "lidar_nav = agrobot_navigation.lidar_nav:main",
        ],
    },
)
