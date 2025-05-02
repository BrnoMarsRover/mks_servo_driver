from setuptools import find_packages, setup

package_name = ('mks_servo_driver')

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
    maintainer='ros',
    maintainer_email='stanislav.svediroh@vut.cz',
    description='Driver for MKS stepper motor drivers',
    license='MIT',
    entry_points={
        'console_scripts': [
            "mks_interface = mks_servo_driver.async_driver:main",
        ],
    },
)
