from setuptools import setup

package_name = 'evaluator_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Logs error and energy metrics for ping-pong arm project',
    license='MIT',
    entry_points={
    'console_scripts': [
        'evaluator_node = evaluator_node.error_efficiency_logger:main',
        'dummy_joint_plot_publisher = evaluator_node.dummy_joint_plot_publisher:main',
        'plotcsv = evaluator_node.plot_csv_data:main',
    ],
},


)
