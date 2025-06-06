from setuptools import find_packages, setup

package_name = 'explorer'

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
    maintainer='batman',
    maintainer_email='nssamyakvarma@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gradient_explorer =  explorer.gradient_explorer:main',
            'cloud_to_grid = explorer.cloud_to_grid:main'
        ],
    },
)
