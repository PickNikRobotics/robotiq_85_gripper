import os
from glob import glob
from setuptools import setup

package_name = 'robotiq_85_driver'
package_subdirectory = 'robotiq_85_driver/driver'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, package_subdirectory],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch/*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stanley Innovation, Inc.',
    maintainer_email='dev@stanleyinnovation.com',
    description='Drivers and nodes related to the Robotiq 85 Gripper',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotiq_85_driver = robotiq_85_driver.robotiq_85_driver:main',
            'single_robotiq_85_action_server = robotiq_85_driver.single_robotiq_85_action_server:main',
            'robotiq_85_test = robotiq_85_driver.robotiq_85_test:main',
            'robotiq_85_test_close = robotiq_85_driver.robotiq_85_test_close:main',
        ],
    },
)