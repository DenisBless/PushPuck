from distutils.core import setup

setup(
    name='PushPuck',
    version='0.1',
    packages=['push_puck'],
    url='',
    license='',
    author='Denis Blessing',
    author_email='',
    description='',
    install_requires=[
        'matplotlib',
        'mujoco_py',
        'numpy',
        'mp_lib @ git+https://git@github.com/maxhuettenrauch/mp_lib@master',
    ]
)
