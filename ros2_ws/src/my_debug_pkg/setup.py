# ~/ros2_ws/src/my_debug_pkg/setup.py
from setuptools import find_packages, setup # find_packages is good practice

package_name = 'my_debug_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']), # Automatically finds your 'my_debug_pkg' inner folder
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='manuel', # Change to your name/email
    maintainer_email='manuel@todo.todo', # Change to your name/email
    description='Package to check environment variables for GStreamer debugging',
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'env_checker = my_debug_pkg.env_check_node:main',
        ],
    },
)
