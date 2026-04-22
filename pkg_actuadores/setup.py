from setuptools import find_packages, setup

package_name = 'pkg_actuadores'

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
            'motor_driver_node = pkg_actuadores.motor_driver_node:main',
            'blade_node = pkg_actuadores.blade_node:main',
            'trimmer_node = pkg_actuadores.trimmer_node:main',
            'buzzer_node = pkg_actuadores.buzzer_node:main',
            'display_node = pkg_actuadores.display_node:main',
        ],
    },
)
