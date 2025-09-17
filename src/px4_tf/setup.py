from setuptools import find_packages, setup

package_name = 'px4_tf'

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
    maintainer='carlson',
    maintainer_email='yichao_gao@u.nus.edu',
    description='TODO: Package description',
    license='BSD-3-Clause',
    # tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_convert = px4_tf.tf_convert:main',
        ],
    },
)
