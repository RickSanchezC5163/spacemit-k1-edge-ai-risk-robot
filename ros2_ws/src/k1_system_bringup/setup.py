import os
from glob import glob

from setuptools import setup

package_name = "k1_system_bringup"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="K1 Robot Team",
    maintainer_email="team@example.invalid",
    description="Unified non-arm bring-up launch package for the K1 robot.",
    license="MIT",
)
