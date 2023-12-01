from setuptools import find_packages, setup

package_name = 'mobile_robot_localization'

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
    maintainer='ros',
    maintainer_email='sbhurat@umich.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                "localization_node = mobile_robot_localization.localization_gps_imu:main",
                "controller_node = mobile_robot_localization.rectangle_drive_controller:main"
        ],
    },
)
