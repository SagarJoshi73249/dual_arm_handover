from setuptools import setup

package_name = 'dual_arm_handover'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sagar Joshi',
    maintainer_email='joshisagar361@gmail.com',
    description='Bimanual manipulation with diffusion policy',
    license='MIT',
)