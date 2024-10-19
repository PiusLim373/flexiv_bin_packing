from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'master_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sesto',
    maintainer_email='sesto@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'master_controller_node = master_controller.master_controller_node:main',
            'stepwise_master_controller_node = master_controller.stepwise_master_controller_node:main',
            'stepwise_master_controller_charuco_node = master_controller.stepwise_master_controller_charuco_node:main',
            'stepwise_master_controller_pcl_node = master_controller.stepwise_master_controller_pcl_node:main',
            'tree_debug = master_controller.tree_debug:main',
            'test_bin_packer_fixed_item = master_controller.test_bin_packer_fixed_item:main',
            'bin_packer_smach = master_controller.bin_packer_smach:main',
        ],
    },
)
