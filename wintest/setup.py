from setuptools import find_packages, setup

package_name = 'wintest'

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
    maintainer='public',
    maintainer_email='scarlet777735@gmail.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'win_data_sub             =  wintest.win_data_sub:main',
            'yaml_to_array_publisher  =  wintest.yaml_to_array_publisher:main',

        ],
    },
)
