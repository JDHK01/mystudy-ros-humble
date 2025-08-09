from setuptools import find_packages, setup

package_name = 'helloworld'

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
    maintainer='root',
    maintainer_email='zhangyiqun@bupt.edu.cn',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 节点定义处
            'helloworld_node = helloworld.helloworld_node:print_helloworld',
            'std_helloworld_node = helloworld.std_helloworld_node:print_helloworld',
        ],
    },
)
