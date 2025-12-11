from setuptools import find_packages
from setuptools import setup

setup(
    name='leo_rover_custom_interface_package',
    version='0.0.0',
    packages=find_packages(
        include=('leo_rover_custom_interface_package', 'leo_rover_custom_interface_package.*')),
)
