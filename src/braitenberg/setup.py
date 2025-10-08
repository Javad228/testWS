from setuptools import find_packages, setup

package_name = 'braitenberg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/braitenberg/launch', ['launch/braitenberg.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jbaghiro',
    maintainer_email='javadbaghirov1@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'braitenberg_node = braitenberg.braitenberg_node:runit',
        ],
    },
)
