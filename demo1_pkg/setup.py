from setuptools import find_packages, setup

package_name = 'demo1_pkg'

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
    maintainer='osman',
    maintainer_email='74795147+osmanaktrk@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "node1 = demo1_pkg.node1:main",
            "publisher1 = demo1_pkg.publisher1:main",
            "subscriber1 = demo1_pkg.subscriber1:main",
            "service1 = demo1_pkg.service1:main",
            "client1 = demo1_pkg.client1:main",
            "client2 = demo1_pkg.client2:main"
        ],
    },
)
