from setuptools import find_packages, setup

setup(
    name='fpv_lab2', version='1.0.0', packages=find_packages(),
    data_files=[('share/ament_index/resource_index/packages', ['resource/fpv_lab2']),
                ('share/fpv_lab2', ['package.xml'])],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Student', maintainer_email='student@example.invalid',
    description='Laboratory 2, variant 15: flight above the maze',
    license='Apache-2.0',
    entry_points={'console_scripts': [
        'flight_test = fpv_lab2.main:main',
    ]},
)
