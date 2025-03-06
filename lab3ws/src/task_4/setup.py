from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'task_4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # This line ensures launch files are properly installed
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'maps'), glob(os.path.join('maps', '*')))

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='me597',
    maintainer_email='cflaherty310@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_4 = task_4.task_4:main',
            'auto_navigator = task_4.auto_navigator:main'
        ],
    },
)
