from setuptools import find_packages, setup

package_name = 'control_pkg'

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
    maintainer='da',
    maintainer_email='da@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'robot_node = control_pkg.robot_node:main',
            'master_node = control_pkg.master_node:main',
            'master_node2 = control_pkg.master_node2:main',
            'master_node3 = control_pkg.master_node3:main',
            'master_node4 = control_pkg.master_node4:main',
            'master_node5 = control_pkg.master_node5:main',
            'master_node6 = control_pkg.master_node6:main',
            'master_node7 = control_pkg.master_node7:main',
            'master_node8 = control_pkg.master_node8:main',
            'master_node_dis = control_pkg.master_node_dis:main',
            'master_node_dis2 = control_pkg.master_node_dis2:main',
            'robot_node_3 = control_pkg.robot_node_3:main',
        ],
    },
)
