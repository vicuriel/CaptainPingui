from setuptools import find_packages, setup

package_name = 'pkg_mision'

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
            'state_machine_node = pkg_mision.state_machine_node:main',
            'nav_selector_node = pkg_mision.nav_selector_node:main',
            'coverage_node = pkg_mision.coverage_node:main',
            'safety_monitor_node = pkg_mision.safety_monitor_node:main',
            'remote_control_node = pkg_mision.remote_control_node:main',
            'app_bridge_node = pkg_mision.app_bridge_node:main',
        ],
    },
)
