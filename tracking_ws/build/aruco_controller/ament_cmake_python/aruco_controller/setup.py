from setuptools import find_packages
from setuptools import setup

setup(
    name='aruco_controller',
    version='0.0.0',
    packages=find_packages(
        include=('aruco_controller', 'aruco_controller.*')),
)
