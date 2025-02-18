from setuptools import find_packages, setup

package_name = 'werdna_agent'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'stable-baselines3', 'gymnasium', 'numpy'],
    zip_safe=True,
    maintainer='andrew',
    maintainer_email='andrewngcheewei@hotmail.com',
    description='TODO: Package description',
    license='Apache 2.0',  # Replace with your actual license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "werdna_agent_node = werdna_agent.werdna_agent_node:main"
        ],
    },
)
