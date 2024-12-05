from setuptools import find_packages, setup

package_name = 'btree'

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
    maintainer='andrek',
    maintainer_email='Hazel.Yu@UDX.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "face_detection_node = btree.face_detection_node:main",
            "track_face = btree.track_face:main",
            "idle = btree.idle:main",
            "asleep = btree.asleep:main",
        ],
    },
)
