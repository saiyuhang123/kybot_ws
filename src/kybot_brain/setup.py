from setuptools import setup
import os
from glob import glob

package_name = 'kybot_brain'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='LLM task scheduler for KYBOT (Qwen function calling)',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'brain_node = kybot_brain.brain_node:main',
        ],
    },
)
