from glob import glob
from setuptools import find_packages, setup


package_name = 'arva_test_protocol'


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/docs', glob('docs/*.md')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Nikolaus Lajtai',
    maintainer_email='nikolaus.lajtai@gmx.at',
    description='Operator-controlled recording and analysis support for ARVA flight experiments.',
    license='TODO',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'arva_test_heading = arva_test_protocol.heading_run:heading_main',
            'arva_test_terrain = arva_test_protocol.terrain_run:terrain_main',
            'arva_test_waypoint = arva_test_protocol.waypoint_run:waypoint_main',
            'arva_test_prepare = arva_test_protocol.run_manager:prepare_main',
            'arva_test_record = arva_test_protocol.run_manager:record_main',
            'arva_test_finish = arva_test_protocol.run_manager:finish_main',
            'arva_test_analyze = arva_test_protocol.analysis:analyze_main',
            'arva_test_mark = arva_test_protocol.event_marker:mark_main',
            'arva_test_aggregate = arva_test_protocol.aggregate:aggregate_main',
            'arva_test_compare = arva_test_protocol.compare:compare_main',
        ],
    },
)
