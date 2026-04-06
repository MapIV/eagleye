from setuptools import setup

package_name = 'eagleye_result_logger'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/result_logger.launch.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Map IV, Inc.',
    maintainer_email='support@map4.jp',
    description='Save eagleye estimation results to CSV for regression comparison',
    license='BSD',
    entry_points={
        'console_scripts': [
            'result_logger = eagleye_result_logger.result_logger_node:main',
        ],
    },
)
