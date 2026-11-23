from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'robotica_practica2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Ainara Maroto',
    maintainer_email='ainara.maroto@opendeusto.es',
    description='Práctica 2: ingesta, calibración y corrección de imágenes con ROS 2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ingesta = robotica_practica2.practica2:main_ingesta',
            'corregir = robotica_practica2.practica2:main_corregir',
        ],
    },
)
