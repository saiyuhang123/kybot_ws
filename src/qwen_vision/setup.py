from setuptools import setup
import os
from glob import glob

package_name = 'qwen_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'picture'), glob('picture/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='Qwen-VL vision perception for KYBOT',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'qwen_vision_node = qwen_vision.qwen_vision_node:main',
            'qwen_point_node = qwen_vision.qwen_point_node:main',
            'depth_value_node = qwen_vision.depth_value_node:main',
            'test_vision_node = qwen_vision.test_vision_node:main',
        ],
    },
)
