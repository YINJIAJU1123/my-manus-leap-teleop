from setuptools import find_packages, setup

package_name = 'telekinesis'

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
    package_data={
        package_name: [
            'leap_hand_mesh_left/*',
            'leap_hand_mesh_right/*',
        ],
    },
    zip_safe=False,
    maintainer='keshaw',
    maintainer_email='lucky7chess@gmail.com',
    description='LEAP hand teleoperation nodes and robot assets',
    license='Proprietary',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_ik = telekinesis.leap_ik:main',
            'leap_ik_manus = telekinesis.leap_ik_manus:main',
            'leap_ik_orin = telekinesis.leap_ik_orin:main',
        ],
    },
)
