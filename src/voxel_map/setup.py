from codecs import open
from os import path
import os
from setuptools import Extension, find_packages, setup
from sys import version_info


def file_content(fpath):
    with open(path.join(here, fpath), encoding='utf-8') as f:
        return f.read()

here = path.abspath(path.dirname(__file__))

# if WINDOWS
if os.name == 'nt':
    import numpy
    BOOST_LIBS = r'D:\boost_1_66_0_src\stage\lib'
    BOOST_ROOT = r'D:\boost_1_66_0_src'
    # there is need to copy the libboost_python3-vc141-mt-x64-1_66 file into boost_python3-vc141-mt-x64-1_66 file
    lib_boost_py = BOOST_LIBS+r'\libboost_python3-vc141-mt-x64-1_66'
    lib_boost_chrono = BOOST_LIBS+r'\libboost_chrono-vc141-mt-x64-1_66'
    lib_boost_system = BOOST_LIBS+r'\libboost_system-vc141-mt-x64-1_66'
    lib_boost_thread = BOOST_LIBS+r'\libboost_thread-vc141-mt-x64-1_66'
    lib_other_boost_py = BOOST_LIBS+r'\boost_python3-vc141-mt-x64-1_66'
    libraries = [lib_boost_py, lib_boost_chrono, lib_boost_system, lib_boost_thread, lib_other_boost_py]
    include_dirs = ['include', numpy.get_include(), BOOST_ROOT]
else:
    # if LINUX
    lib_boost_py = 'boost_python-py%i%i' % version_info[:2]
    lib_boost_chrono = 'boost_chrono'
    lib_boost_system = 'boost_system'
    lib_boost_thread = 'boost_thread'
    libraries = [lib_boost_py, lib_boost_chrono, lib_boost_system, lib_boost_thread]
    include_dirs = ['include']

setup(
    name='voxel_map',
    version='0.0.1',
    description='Simple C++ header-only library with Matlab and Python interfaces for dealing with voxel maps.',
    long_description=file_content('README.md'),
    url='https://bitbucket.org/tpetricek/voxel_map',
    author='Tomas Petricek',
    author_email='tpetricek@gmail.com',
    license='MIT',

    # See https://pypi.python.org/pypi?%3Aaction=list_classifiers
    classifiers=[
        'Development Status :: 3 - Alpha',
        'Intended Audience :: Developers',

        'Intended Audience :: Information Technology',
        'Intended Audience :: Science/Research',

        'Topic :: Software Development :: Build Tools',
        'Topic :: Multimedia :: Graphics :: 3D Modeling',
        'Topic :: Multimedia :: Graphics :: 3D Rendering',

        'License :: OSI Approved :: MIT License',

        'Programming Language :: Python :: 2',
        'Programming Language :: Python :: 2.7',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.5',
        'Programming Language :: Python :: 3.6',
        'Programming Language :: Python :: 3.7',
    ],
    keywords='voxel map volumetric data',
    packages=find_packages(),
    install_requires=['numpy'],
    ext_modules=[
        Extension(
            name='voxel_map',
            sources=['python/voxel_map_module.cpp'],
            include_dirs=include_dirs,
            libraries=libraries,
            extra_compile_args=['-fopenmp'],
            extra_link_args=['-lgomp']
        )
    ]
)
