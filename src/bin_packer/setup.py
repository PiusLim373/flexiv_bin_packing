from setuptools import find_packages, setup

package_name = 'bin_packer'

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
    maintainer='sesto',
    maintainer_email='piuslim373@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bin_packer_node = bin_packer.bin_packer_node:main',
            'test = bin_packer.matplotlib_ros:main'
        ],
    },
)
