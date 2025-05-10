from setuptools import find_packages
from setuptools import setup

setup(
    name='inverse_kinematics_node',
    version='0.0.0',
    packages=find_packages(
        include=('inverse_kinematics_node', 'inverse_kinematics_node.*')),
)
