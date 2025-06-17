from setuptools import find_packages, setup

package_name = 'fbot_manipulator_tools'

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
    maintainer='luizlost',
    maintainer_email='luized_sparvoli@hotmail.com',
    description='A set of tools for the fbot_manipulator, including a node to save the wx200 arm poses.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'manipulator_saver = fbot_manipulator_tools.save_wx200_arm_pose:main',
        ],
    },
)
