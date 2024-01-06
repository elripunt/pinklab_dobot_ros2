from setuptools import find_packages, setup
import glob, os

package_name = 'dobot_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob.glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/param', glob.glob(os.path.join('param', '*.yaml'))),
        ('lib/' + package_name + '/api', glob.glob(os.path.join('api', '*.py')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mkh',
    maintainer_email='kyung133851@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dashboard_server=dobot_ros2.dashboard_server:main",
            "dashboard_client=dobot_ros2.dashboard_client:main",
            "move_goal_client=dobot_ros2.move_goal_client:main",
        ],
    },
)
