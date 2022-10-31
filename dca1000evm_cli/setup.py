from setuptools import setup
import os
from glob import glob

package_name = 'dca1000evm_cli'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
                                            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'cli'), glob('cli/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='mohamed.abdelaziz@zal.aero',
    description='ROS wrapper for the DCA1000EVM CLI tool',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dca1000evm_cli_node = dca1000evm_cli.dca1000evm_cli_node:main'
        ],
    },
)
