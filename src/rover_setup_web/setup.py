import os

from setuptools import find_packages, setup


def data_files_from_tree(source_dir: str, install_dir: str):
    data_files = []
    for root, _dirs, files in os.walk(source_dir):
        selected = [
            os.path.join(root, name)
            for name in files
            if not name.startswith('.')
        ]
        if not selected:
            continue
        relative = os.path.relpath(root, source_dir)
        target = install_dir if relative == '.' else os.path.join(install_dir, relative)
        data_files.append((target, selected))
    return data_files


package_name = 'rover_setup_web'
setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    # app.py serves the UI from os.path.dirname(__file__)/static, so static/
    # must be installed INSIDE the python package (site-packages), not only
    # under share/: a plain non-symlink colcon build would otherwise 404 on /.
    include_package_data=True,
    package_data={package_name: ['static/*', 'static/**/*']},
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ] + data_files_from_tree(
        os.path.join(package_name, 'static'),
        os.path.join('share', package_name, 'static')),
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Rover Team', maintainer_email='maintainer@example.com',
    description='Standalone Flask web app for first-time VESC/CAN drivetrain setup.',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'setup_web = rover_setup_web.app:main',
    ]},
)
