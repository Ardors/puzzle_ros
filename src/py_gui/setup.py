import os
from glob import glob

from setuptools import setup

package_name = 'py_gui'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'ico'), glob('ico/*.ico')),
        (os.path.join('share', package_name, 'images'), glob('images/*.png')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='fernandasuarez',
    maintainer_email='fernanda.suarez@alumnos.upm.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gui = py_gui.py_gui:main',
        ],
    },
)
