from setuptools import find_packages, setup
import glob

package_name = 'warehouse_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/models', glob.glob('warehouse_perception/models/*.pt')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='moharaga',
    maintainer_email='moharaga@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'yolo_detector_node = warehouse_perception.yolo_detector_node:main',
            'capture_images = warehouse_perception.capture_images:main',
        ],
    },
)
