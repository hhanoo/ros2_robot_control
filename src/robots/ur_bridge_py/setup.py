from setuptools import find_packages, setup

package_name = 'ur_bridge_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='woo980711@gmail.com',
    description='ROS2 Bridge for controlling UR robot via TCP socket (Python)',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest',],
    },
    entry_points={
        'console_scripts': ['ur_bridge_node = ur_bridge_py.ur_bridge_node:main'],
    },
)
