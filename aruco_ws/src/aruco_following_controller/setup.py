from setuptools import find_packages, setup

package_name = 'aruco_following_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/param', ['param/controller_params.yaml']),
        ('share/' + package_name + '/launch', ['launch/controller.launch.py']),
        ('share/' + package_name + '/launch', ['launch/static_tf.launch.py'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sorawit-i',
    maintainer_email='sorawit.i@obodroid.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'controller = aruco_following_controller.robot_controller:main'
        ],
    },
)