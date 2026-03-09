from setuptools import find_packages, setup
from glob import glob

package_name = 'robot_dance'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pravin-raj',
    maintainer_email='pravinraj2054@gmail.com',
    description='H1 Robot Dance Choreographer',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'dance_gui = robot_dance.dance_gui:main',
        ],
    },
)
