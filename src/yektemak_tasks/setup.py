from glob import glob

from setuptools import setup

package_name = "yektemak_tasks"

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ali",
    maintainer_email="haydar.ali61@hotmail.com",
    description="Yektemak palet gorevleri: docking, lift, gorev yoneticisi",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "map_explorer = yektemak_tasks.map_explorer:main",
            "mission_manager = yektemak_tasks.mission_manager:main",
        ],
    },
)
