
from setuptools import setup, find_packages
import platform

setup(
    name='bao',
    version='1.0',
    packages=['bao'],
    package_dir={'': 'src'},
    license='mwj 1.0',
    description='jiu xie yi 1.0',
    author='Maweijie',
    author_email='maweijie@jacode.cn',
    install_requires=['pyserial','keyboard']
)
