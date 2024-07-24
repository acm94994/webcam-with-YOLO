from setuptools import find_packages, setup

package_name = 'webcam_yolo'

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
    maintainer='acm',
    maintainer_email='chandramouliaravind05@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'webcam_publisher = webcam_yolo.webcam_pub:main',
            'yolov8_subscriber = webcam_yolo.webcam_sub:main',
            'rng = webcam_yolo.rng:main',
            'combine_array = webcam_yolo.combine_array:main',
        ],
    },
)
