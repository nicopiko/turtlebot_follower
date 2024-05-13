from setuptools import find_packages, setup

package_name = 'turtlebot_follower'

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
    maintainer='nicolai',
    maintainer_email='nstall22@student.aau.dk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_subscriber = turtlebot_follower.camera_subscriber:main',
            'lidar_subscriber = turtlebot_follower.lds_subscriber:main',
            'angular_regulator = turtlebot_follower.angular_reg:main',
            'kp_tuning = turtlebot_follower.kp_tuning:main',
            'straight_test = turtlebot_follower.straight_test:main',
            'move_controller = turtlebot_follower.move_controller:main',
            'stop_move = turtlebot_follower.stop_move:main',
            'angular_test = turtlebot_follower.angular_test:main'
        ],
    },
)
