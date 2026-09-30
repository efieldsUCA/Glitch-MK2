from setuptools import find_packages, setup
from pathlib import Path

package_name = 'glitchmk2_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (
            str(Path("share") / package_name / "config"),
            [str(file) for file in Path("config").glob("*.yaml")],
        ),
        (
            str(Path("share") / package_name / "launch"),
            [str(file) for file in Path("launch").glob("*_launch.py")],
        ),
        (
            str(Path("share") / package_name / "rviz"),
            [str(file) for file in Path("rviz").glob("*.rviz")],
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pbd0',
    maintainer_email='pbd0@todo.todo',
    description='TODO: Package description',
    license='GPL-3.0-only',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pico_interface = glitchmk2_bringup.pico_interface:main',
            'basic_pico_interface = glitchmk2_bringup.basic_pico_interface:main',
            'aruco_detector = glitchmk2_bringup.aruco_detector:main',
            'chronos_recorder = glitchmk2_bringup.chronos_recorder:main',
            'chronos_pilot = glitchmk2_bringup.chronos_pilot:main',
        ],
    },
)
