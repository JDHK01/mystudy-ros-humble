from setuptools import find_packages, setup

package_name = 'topic_demo'

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
            'listener = topic_demo.topic_subscriber_node:receive_helloworld',
            'talker = topic_demo.topic_publisher_node:print_helloworld',
            'customized_listener = topic_demo.customized_subscriber_node:receive_position'
            'customized_talker = topic_demo.customized_publisher_node:print_position'
        ],
    },
)
