from setuptools import find_packages, setup

package_name = 'pkg_navegacion'

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
            'odometry_node = pkg_navegacion.odometry_node:main',
            'geofence_node = pkg_navegacion.geofence_node:main',
            'map_node = pkg_navegacion.map_node:main',
            'parallel_nav_node = pkg_navegacion.parallel_nav_node:main',
            'random_nav_node = pkg_navegacion.random_nav_node:main',
            'perimeter_nav_node = pkg_navegacion.perimeter_nav_node:main',
            'obstacle_avoid_node = pkg_navegacion.obstacle_avoid_node:main',
        ],
    },
)
