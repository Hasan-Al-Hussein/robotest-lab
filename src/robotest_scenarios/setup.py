# Copyright 2026 Hasan Ahmed
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from glob import glob

from setuptools import find_packages, setup

PACKAGE_NAME = 'robotest_scenarios'


setup(
    name=PACKAGE_NAME,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + PACKAGE_NAME]),
        ('share/' + PACKAGE_NAME, ['package.xml', 'README.md', 'LICENSE']),
        ('share/' + PACKAGE_NAME + '/schema', glob('schema/*.json')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Hasan Ahmed',
    description='Deterministic scenario actors and collision positive control.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'scenario_controller = robotest_scenarios.scenario_controller:main',
            'contact_control_driver = robotest_scenarios.contact_control_driver:main',
        ],
    },
)
