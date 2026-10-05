from setuptools import find_packages
from setuptools import setup

setup(
    name='robmovil_msgs',
    version='0.0.0',
    packages=find_packages(
        include=('robmovil_msgs', 'robmovil_msgs.*')),
)
