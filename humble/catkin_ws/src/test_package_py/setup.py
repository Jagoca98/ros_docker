from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'test_package_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files.
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abarrera',
    maintainer_email='abarrera@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'test_node_py = test_package_py.test_node_py:main',
            'talker = test_package_py.publisher_member_function:main',
            'listener = test_package_py.subscriber_member_function:main',
            'talkerfib = test_package_py.publisher_member_function_fibonacci:main',
            'listenerfib = test_package_py.subscriber_member_function_fibonacci:main',
            'listenerImg = test_package_py.subscriber_image:main',
            'talkerImg = test_package_py.publisher_image:main',
        ],
    },
)
