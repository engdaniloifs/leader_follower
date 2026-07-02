from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'leader_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
    ('share/ament_index/resource_index/packages',
        ['resource/' + package_name]),
    ('share/' + package_name, ['package.xml']),
    (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = leader_controller.pure_pursuit:main',
            'NMPC = follower_controller.NMPC:main',
            'GMPC_phi = follower_controller.GMPC_phi:main',
            'GMPC_ackermann = follower_controller.GMPC_ackermann:main',
            'leader_trajectory_eight = leader_controller.leader_trajectory_eight:main',
            'leader_trajectory = leader_controller.leader_trajectory:main',
            'planner = follower_controller.path_planner:main',
            'FBlinear = follower_controller.FBlinear:main',
            'EKF = state_estimation.EKF:main',
        ],
    },
)
