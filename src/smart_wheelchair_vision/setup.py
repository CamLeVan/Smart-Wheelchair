from setuptools import setup

package_name = 'smart_wheelchair_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'ultralytics', 'opencv-python'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@todo.todo',
    description='Vision and AI tracking package for Smart Wheelchair',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'human_tracker = smart_wheelchair_vision.human_tracker:main'
        ],
    },
)
