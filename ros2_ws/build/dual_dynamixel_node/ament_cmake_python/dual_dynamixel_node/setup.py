from setuptools import find_packages
from setuptools import setup

setup(
    name='dual_dynamixel_node',
    version='0.0.0',
    packages=find_packages(
        include=('dual_dynamixel_node', 'dual_dynamixel_node.*')),
)
