from setuptools import find_packages, setup

package_name = 'pkg_sensores'

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
            'gps_node = pkg_sensores.gps_node:main',
            'imu_node = pkg_sensores.imu_node:main',
            'ultrasonic_node = pkg_sensores.ultrasonic_node:main',
            'bumper_node = pkg_sensores.bumper_node:main',
            'encoder_node = pkg_sensores.encoder_node:main',
            'lifted_node = pkg_sensores.lifted_node:main',
            'battery_node = pkg_sensores.battery_node:main',
        ],
    },
)
