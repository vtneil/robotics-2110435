from setuptools import find_packages, setup

package_name = 'keyboard_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khadas',
    maintainer_email='tharittapol.t@obodroid.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_control = keyboard_control.keyboard_control:main',
            'vel_mux = keyboard_control.vel_mux:main'
        ],
    },
)
