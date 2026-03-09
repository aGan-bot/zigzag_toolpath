from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'zigzag_toolpath'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.launch.py')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='as',
    maintainer_email='agans@outlook.com',
    description='creating zigzag toolpath around tf acorrding ...',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'zigzag_node = zigzag_toolpath.zigzag_node:main',
            'zigzag_tf_node = zigzag_toolpath.zigzag_tf_node:main',
            'zigzag_debug_node = zigzag_toolpath.zigzag_debug_node:main',
        ],
    },
)
