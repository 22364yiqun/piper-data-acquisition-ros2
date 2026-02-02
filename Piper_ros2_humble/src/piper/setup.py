from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'piper'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yiqun',
    maintainer_email='2236403741@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'piper_start_master_node = piper.piper_start_master_node:main',
            'piper_start_slave_node = piper.piper_start_slave_node:main',
            'piper_read_master_node = piper.piper_read_master_node:main',
            'piper_start_ms_node = piper.piper_start_ms_node:main',
        ],
    },
)
