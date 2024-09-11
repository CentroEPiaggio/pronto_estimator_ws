from setuptools import find_packages, setup

package_name = 'fatigue_test'

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
    maintainer='JacopINO, JakoPunk',
    maintainer_email='cionix90@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # "qb_test_node = fatigue_test.fatique_test_node:main"
            "qb_test_node = fatigue_test.JumpingTest_002:main"
            # "qb_test_node = fatigue_test.JumpingTest_002_ok_jump:main"
        ],
    },
)
