from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['cw3q2'],
    package_dir={'': 'src'}
)

setup(**d)
