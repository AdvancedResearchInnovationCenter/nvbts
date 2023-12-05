from distutils.core import setup
import pkg_resources
from setuptools import setup, find_packages
import os
    
setup(
    name='nvbts',
    version='0.1',
    description='Neuromorphic vision-based tactile sensing (ARIC)',
    author='Hussain Sajwani',
    author_email='',
    url='https://github.com/AdvancedResearchInnovationCenter/nvbts',
    install_requires=[
        str(r)
        for r in pkg_resources.parse_requirements(
            open(os.path.join(os.path.dirname(__file__), "requirements.txt"))
        )
    ] 
)