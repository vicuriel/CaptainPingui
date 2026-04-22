from setuptools import find_packages, setup

package_name = 'pkg_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='captainp',
    maintainer_email='captainp@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_node = pkg_vision.camera_node:main',
            'grass_detector_node = pkg_vision.grass_detector_node:main',
            'obstacle_vision_node = pkg_vision.obstacle_vision_node:main',
            'tilt_detector_node = pkg_vision.tilt_detector_node:main',
        ],
    },
)
