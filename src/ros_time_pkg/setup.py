from setuptools import find_packages, setup

package_name = 'ros_time_pkg'

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
            'time_publisher = ros_time_pkg.time_publisher:main',
            'time_subscriber = ros_time_pkg.time_subscriber:main',
        ],
    },
)
