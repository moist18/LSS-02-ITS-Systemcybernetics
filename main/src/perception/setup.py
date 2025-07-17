from setuptools import setup

package_name = 'perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='penship',
    maintainer_email='arielsulton26@gmail.com',
    description='perception package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'record = perception.record:main',
            'vision = perception.vision:main',
            'det_gps = perception.det_gps:main'
        ],
    },
)