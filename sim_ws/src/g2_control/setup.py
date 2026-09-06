from setuptools import find_packages, setup
from glob import glob

package_name = 'g2_control'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wxf',
    maintainer_email='wxf@todo.todo',
    description='G2 仿真控制: 舵轮底盘运动学/里程计/虚拟激光/相对运动/导航中继',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'swerve_drive_node = g2_control.swerve_drive_node:main',
            'laser_sim_node = g2_control.laser_sim_node:main',
            'chassis_relative_move_node = g2_control.chassis_relative_move_node:main',
            'nav_goal_relay_node = g2_control.nav_goal_relay_node:main',
        ],
    },
)
