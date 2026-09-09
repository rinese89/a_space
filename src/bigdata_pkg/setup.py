from setuptools import find_packages, setup

package_name = 'bigdata_pkg'

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
    maintainer='rinese',
    maintainer_email='rinese@etsii.upv.es',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'a_space_mongo_macro_ingestor = bigdata_pkg.a_space_mongo_macro_ingestor:main',
        ],
    },
)
