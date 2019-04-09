from setuptools import find_packages
from setuptools import setup

package_name = 'ros2_xmlrpc_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Aarush Gupta',
    author_email='aarush.gupta@hopetechnik.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description=(
        'Python package for ros2 interface with xmlrpc '
        'currently acts as a xmlrpc client only, not added the server yet'
    ),
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            #'ros2_xmlrpc_client = ros2_xmlrpc_interface.ros2_xmlrpc_client:main',
            'ros2_xmlrpc_client_new = ros2_xmlrpc_interface.ros2_xmlrpc_client_new:main',
            'task_info_generater = ros2_xmlrpc_interface.task_info_generater:main'
        ],
    },
)
