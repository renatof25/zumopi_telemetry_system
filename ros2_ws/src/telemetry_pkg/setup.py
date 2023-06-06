from setuptools import setup

package_name = 'telemetry_pkg'

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
    maintainer='zumo',
    maintainer_email='zumo@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "wifi_stats_node = telemetry_pkg.wifi_stats:main",
            "serial_interface_node = telemetry_pkg.serial_interface:main",
            "ups_reading_node = telemetry_pkg.ups_reading:main",
            "camera_node = telemetry_pkg.camera:main",
            "gui_node = telemetry_pkg.gui:main",
            "serial_interface_generator_node = telemetry_pkg.serial_interface_gen:main",
            "ups_generator_node = telemetry_pkg.ups_gen:main",
            "camera_generator_node = telemetry_pkg.camera_gen:main",
            "wifi_generator_node = telemetry_pkg.wifi_gen:main"
        ],
    },
)
