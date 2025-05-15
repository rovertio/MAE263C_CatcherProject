from setuptools import find_packages
from setuptools import setup

setup(
    name='joint_positions_node',
    version='0.0.0',
    packages=find_packages(
        include=('joint_positions_node', 'joint_positions_node.*')),
)
