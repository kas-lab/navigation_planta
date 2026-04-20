from setuptools import find_packages, setup

setup(
    name='navigation_planta',
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=['setuptools'],
    author='Gustavo Rezende',
    author_email='g.rezendesilva@tudelft.nl',
    description='PDDL-TOMASys ontology for a navigation scenario',
    license='Apache-2.0',
)
