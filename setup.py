from setuptools import setup

package_name = 'dbot_driver'

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
    maintainer='dean',
    maintainer_email='deanmacasilhig@gmail.com',
    description='Driver of multiple ODrives for the DBOT robot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dbot_driver_node = dbot_driver.dbot_controller_node:main'
        ],
    },
)
