from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'dog12a04_ik_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='umas',
    maintainer_email='1970313791@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ik_controller = dog12a04_ik_controller.scripts.control_ik:main',
            'foot_trajectory_publisher = dog12a04_ik_controller.scripts.pub_foot_traj:main',
        ],
    },
)