from glob import glob
import os
from setuptools import find_packages, setup

package_name = 'rover_vesc_driver'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Rover Team',
    maintainer_email='maintainer@example.com',
    description='cmd_vel to four VESC controllers over CAN with telemetry feedback.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        # Мост к демону rover-motord (штатный режим, base_driver.type: vesc).
        'vesc_bridge_node = rover_vesc_driver.vesc_bridge_node:main',
        # Прямой доступ к CAN (откат, base_driver.type: vesc_direct;
        # перед запуском остановите rover-motord).
        'vesc_driver_node = rover_vesc_driver.vesc_driver_node:main',
    ]},
)
