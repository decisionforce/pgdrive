import sys
from distutils.core import setup

from setuptools import find_packages

assert sys.version_info.major == 3 and sys.version_info.minor >= 6, "python version >= 3.6 is required"

setup(
    name="PGDrive",
    version="0.1.0",
    description="PGDrive: an open-ended driving simulator with infinite scenes",
    url="https://github.com/decisionforce/pgdrive",
    author="Quanyi Li, Zhenghao Peng",
    author_email="liquanyi@bupt.edu.cn, pengzh@ie.cuhk.edu.hk",
    packages=find_packages(),
    install_requires=[
        "gym",
        "numpy<=1.19.3",
        "matplotlib",
        "pandas",
        "pygame==2.0.0",
        "yapf==0.30.0",
        "seaborn",
        "panda3d==1.10.5",
        "panda3d-gltf",
        "panda3d-simplepbr",
        "pillow"
    ],
    include_package_data=True,
    license="Apache License 2.0"
)
