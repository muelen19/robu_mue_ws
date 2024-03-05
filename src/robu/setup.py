from setuptools import find_packages, setup

package_name = 'robu'

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
    maintainer='robu',
    maintainer_email='muelen19@htl-kaindorf.at',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wallfollower = robu.ex10_wallfollower:main',
            'mypublisher = robu.publisher:main',
            'mysubscriber = robu.subscriber:main',
            'myparameter = robu.ex11_parameter:main',
            'fibonacci_server = robu.ex12_fibonacci_server:main'
            'fibonacci_client = robu.ex12_fibonacci_client:main'
        ],
    },
)
