from setuptools import find_packages, setup

package_name = 'trash_mission'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/trash_mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nvidia',
    maintainer_email='nvidia@todo.todo',
    description='巡检自动抓取：车前 D435 实时检测与测距',
    license='MIT',
    entry_points={
        'console_scripts': [
            'front_perception_node = trash_mission.front_perception_node:main',
        ],
    },
)
