from setuptools import setup, find_packages
from catkin_pkg.python_setup import generate_distutils_setup

packages = find_packages("python/src")

d = generate_distutils_setup(packages=packages, package_dir={"": "python/src"})

setup(**d)
