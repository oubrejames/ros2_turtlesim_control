from setuptools import setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', "launch/waypoints_launch.xml", "config/colors.yaml"]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='oubre',
    maintainer_email='oubrejames@gmail.com',
    description='Control the turtle in turtlesim to go to certain waypoints.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypointnode = turtle_control.waypoint:main'
            # 'talker = py_pubsub.publisher_member_function:main'
            # 'nodeName = packageName.executable:main'
        ],
    },
)
