from glob import glob

from setuptools import setup

package_name = 'lunabotdemo'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob('launch/*')),
        ('share/' + package_name, glob('urdf/*')),
        ('share/' + package_name, glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sam Rooney',
    maintainer_email='the.sam.rooney@gmail.com',
    description='Lunabot demonstration scripts',
    license='Not Applicable',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'state_publisher = lunabotdemo.state_publisher:main'
        ],
    },
)