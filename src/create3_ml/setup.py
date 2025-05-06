from setuptools import find_packages, setup

package_name = 'create3_ml'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='85811392+Lance-Town@users.noreply.github.com',
    description='TODO: Package description',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = create3_ml.create3_action_server:main',
            'client = create3_ml.create3_action_client:main',
            'status = create3_ml.create3_status_node:main',
            'modbus = create3_ml.pymodbus_server_test_lance:main',
            'control = create3_ml.create3_control_node:main',
        ],
    },
)
