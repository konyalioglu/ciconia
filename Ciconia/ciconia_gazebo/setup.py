from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

package_name = 'ciconia_gazebo'


d = generate_distutils_setup(
    packages= package_name,
    scripts = ['scripts'],
    package_dir={'': 'src'}
)

setup(**d)
