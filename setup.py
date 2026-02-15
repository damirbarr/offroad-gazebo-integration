from setuptools import setup

package_name = 'offroad_gazebo_integration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Damir Barr',
    maintainer_email='damir@ottopia.tech',
    description='Off-road autonomy simulation integration using Gazebo Sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gazebo_adapter_node = gazebo_adapter.adapter:main',
            'terrain_generator = gazebo_adapter.terrain:main',
            'bridge_manager = gazebo_adapter.bridge:main',
        ],
    },
)
