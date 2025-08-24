from setuptools import find_packages, setup

package_name = 'agro-driver-new'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'hoverboard_driver = agro_driver_new.hoverboard_driver:main',
            'manual_wheel_control = agro_driver_new.manual_wheel_control:main',
        ],
    },
)