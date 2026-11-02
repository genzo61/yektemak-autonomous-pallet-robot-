import os
from glob import glob

from setuptools import setup

package_name = "yektemak_fleet"


def tree(src, dst):
    out = []
    for root, _, files in os.walk(src):
        if files:
            out.append((os.path.join(dst, os.path.relpath(root, src)), [os.path.join(root, f) for f in files]))
    return out


setup(
    name=package_name,
    version="0.3.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ] + tree("web", "share/" + package_name + "/web"),
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ali",
    maintainer_email="haydar.ali61@hotmail.com",
    description="Yektemak filo yonetimi: gorev atama, bolge kilitleri, operator paneli",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": [
        "fleet_manager = yektemak_fleet.fleet_manager:main",
        "dashboard_server = yektemak_fleet.dashboard_server:main",
    ]},
)
