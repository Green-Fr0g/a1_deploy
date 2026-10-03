from setuptools import find_packages, setup

package_name = 'a1_mujoco_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/sim.yaml']),
    ],
    install_requires=['setuptools', 'PyYAML'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'a1_mujoco_sim = a1_mujoco_sim.cli:main',
        ],
    },
)
