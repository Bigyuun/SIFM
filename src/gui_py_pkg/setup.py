from setuptools import find_packages, setup
import glob
import os

package_name = 'gui_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='daeyun',
    maintainer_email='bigyun9375@gmail.com',
    description='gui-for-control',
    license='Apache-License-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui_node = gui_py_pkg.gui_node:main',
            'rs_node = gui_py_pkg.rs_read:main',
            'test = gui_py_pkg.push_test:main'
        ],
    },
)
