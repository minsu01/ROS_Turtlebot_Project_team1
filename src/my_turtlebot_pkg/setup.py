from setuptools import find_packages, setup

package_name = 'my_turtlebot_pkg'

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
    maintainer='noh0906',
    maintainer_email='noh0906@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
          'move_turtle_pub = my_turtlebot_pkg.move_turtle_pub:main',
          'detect_obstacle_pkg = my_turtlebot_pkg.detect_obstacle_pkg:main',
          'move_key_turtle = my_turtlebot_pkg.move_key_turtle:main',
          'turtlebot_pose = my_turtlebot_pkg.turtlebot_pose:main',
          'turtlebot_server = my_turtlebot_pkg.turtlebot_server:main',
          'turtlebot_client = my_turtlebot_pkg.turtlebot_client:main',
          'move_hand = my_turtlebot_pkg.move_hand:main',
          'yolo = my_turtlebot_pkg.yolo:main',
        ],
    },
)
