from setuptools import find_packages, setup

package_name = 'ets_dp_nav'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup_follow.launch.py']),
        ('share/' + package_name + '/config', [
            'config/follow_point_bt.xml',
            'config/nav2_params.yaml'
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ets',
    maintainer_email='123sashanksasi321@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'target_generator = ets_dp_nav.target_generator:main',
            'nav2_to_px4_bridge = ets_dp_nav.nav2_to_px4_bridge:main',
            'px4_tf_publisher = ets_dp_nav.px4_tf_publisher:main',
            'goal_relay = ets_dp_nav.goal_relay:main',
        ],
    },
)
