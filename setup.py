from setuptools import find_packages, setup

package_name = 'pose_annotation'

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
    maintainer='neo',
    maintainer_email='arash.akhlaghi.d@gmail.com',
    description='Package for gate and robot position transformation and annotation.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = pose_annotation.image_publisher:main',
            'pose_annotation_node = pose_annotation.pose_annotation_node:main',
            'transform = pose_annotation.transform:main',
            'lidar_3d = pose_annotation.lidar_3d:main'
        ],
    },
)
