from setuptools import find_packages, setup

package_name = 'joint_space_py'

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
    maintainer='surya-cogman',
    maintainer_email='surya.roboengr@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'time_based = joint_space_py.time_based:main',
            'velocity_based = joint_space_py.velocity_based:main',
            'trapezoidal_motion = joint_space_py.trapezoidal_motion:main',
        ],
    },
)
