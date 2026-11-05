from glob import glob

from setuptools import setup

package_name = "yektemak_gnss"
setup(
    name=package_name,
    version="0.4.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="ali",
    maintainer_email="haydar.ali61@hotmail.com",
    description="Yektemak RTK-GNSS simulatoru ve GNSS konum fuzyonu",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={"console_scripts": ["rtk_simulator = yektemak_gnss.rtk_simulator:main"]},
)
