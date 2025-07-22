from setuptools import find_packages, setup
import os
import glob


package_name = 'turtle_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='noh0906',
    maintainer_email='noh0906@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'turtle_pub = turtle_pkg.turtle_pub:main',
          'turtle_cmd_and_pose = turtle_pkg.turtle_cmd_and_pose:main',
          'my_service_server = turtle_pkg.my_service_server:main',
          'dist_turtle_action_server = turtle_pkg.dist_turtle_action_server:main',
          'my_subscriber = turtle_pkg.my_subscriber:main',
          'my_multi_thread = turtle_pkg.my_multi_thread:main',
          'turtlesim_and_teleop.launch = turtle_pkg.turtlesim_and_teleop.launch:main'

        ],
    },
)
