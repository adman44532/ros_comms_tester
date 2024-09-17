import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'comms_tester'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][xma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adman',
    maintainer_email='adman44532@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'endpoint_emulator = comms_tester.endpoint_emulator:main',
            'simple_string_rtt = comms_tester.testers.simple_string_rtt:main',
            'large_payload_rtt = comms_tester.testers.large_payload_rtt:main',
            'increasing_payload_rtt = comms_tester.testers.increasing_payload_rtt:main',
            'custom_message_rtt = comms_tester.testers.custom_message_rtt:main',
        ],
    },
)
