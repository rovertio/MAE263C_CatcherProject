from setuptools import setup

package_name = 'joint_plotter'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    py_modules=[],
    install_requires=['setuptools', 'matplotlib'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@your.email',
    description='Plots arm end-effector XY over time via get_xy service',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joint_plotter = joint_plotter.joint_plotter:main',
        ],
    },
)

