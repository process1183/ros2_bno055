from setuptools import setup

package_name = 'ros2_bno055'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Josh Gadeken',
    maintainer_email='ros@steelcaverobotics.com',
    description='ROS2 node for the Bosch BNO055 IMU',
    license='GNU General Public License version 3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bno055 = ros2_bno055.bno055:main'
        ],
    },
)
