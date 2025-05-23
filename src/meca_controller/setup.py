from setuptools import setup, find_packages

package_name = 'meca_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jessicamyers',
    maintainer_email='myersjm@rose-hulman.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "meca_driver = meca_controller.meca_driver:main",
            "meca_driver_test = meca_controller.meca_driver_test:main",
            "meca_control = meca_controller.meca_control:main",
            "test_driver = meca_controller.test_driver:main",
            "test_command = meca_controller.test_command:main"
        ],
    },
)
