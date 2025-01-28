from setuptools import find_packages, setup

package_name = 'nakama_franka_interaction_buttons'

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
    maintainer='Jelle Hierck',
    maintainer_email='j.j.hierck@student.utwente.nl',
    description='Nodes to trigger actions based on the HMI Interaction Buttons for Nakama Franka',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bag_recorder_manager = nakama_franka_interaction_buttons.bag_recorder_manager:main'
        ],
    },
)
