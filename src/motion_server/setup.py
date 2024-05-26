from setuptools import find_packages, setup

package_name = 'motion_server'
setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['library/flexivrdk.cp38-win_amd64.pyd']),
        ('share/' + package_name, ['library/flexivrdk.cp310-win_amd64.pyd']),
        ('share/' + package_name, ['library/flexivrdk.cpython-38-aarch64-linux-gnu.so']),
        ('share/' + package_name, ['library/flexivrdk.cpython-38-darwin.so']),
        ('share/' + package_name, ['library/flexivrdk.cpython-38-x86_64-linux-gnu.so']),
        ('share/' + package_name, ['library/flexivrdk.cpython-310-aarch64-linux-gnu.so']),
        ('share/' + package_name, ['library/flexivrdk.cpython-310-darwin.so']),
        ('share/' + package_name, ['library/flexivrdk.cpython-310-x86_64-linux-gnu.so']),
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
            'motion_server_node = motion_server.motion_server_node:main'
        ],
    },
)
