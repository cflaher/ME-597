from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line ensures launch files are properly installed
        (os.path.join('share', package_name, 'launch'), glob('launch/*launch.[pxy][yma]*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cian_Flaherty',
    maintainer_email='cflaherty310@gmail.com',
    description='Custom service server and service client node',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'talker = task_2.publisher_member_function:main',
            'service = task_2.service:main',
            'client = task_2.client:main',
        ],
    },
)