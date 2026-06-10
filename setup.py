from setuptools import find_packages, setup

package_name = 'qcar2_controller'

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
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = qcar2_controller.pure_pursuit:main',
            'NMPC = qcar2_controller.NMPC:main',
            'GMPC_phi = qcar2_controller.GMPC_phi:main',
            'GMPC_ackermann = qcar2_controller.GMPC_ackermann:main',
            'leader_trajectory_eight = qcar2_controller.leader_trajectory_eight:main',
            'leader_trajectory = qcar2_controller.leader_trajectory:main',
            'planner = qcar2_controller.path_planner:main',
            'FBlinear = qcar2_controller.FBlinear:main',
        ],
    },
)
