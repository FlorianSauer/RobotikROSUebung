import os
import platform
import sys

try:
    from setuptools import setup
    from setuptools import Extension
except ImportError:
    from distutils.core import setup
    from distutils.extension import Extension
from Cython.Build import cythonize
from Cython.Compiler import Options

# Cythonize options
Options.docstrings = True


# scan the 'dvedit' directory for extension files, converting
# them to extension names in dotted notation
def scandir(dir, files=[]):
    for file in os.listdir(dir):
        path = os.path.join(dir, file)
        if os.path.isfile(path) and path.endswith(".pyx"):
            files.append(path.replace(os.path.sep, ".")[:-4])
        elif os.path.isdir(path):
            scandir(path, files)
    return files


# generate an Extension object from its dotted name
def makeExtension(extName):
    fopenmp = '-fopenmp'
    include_dirs = []

    extNameRoot = extName.split(".")[0]

    # include path to .h files (stdint.h)
    if platform.system() == 'Windows':
        fopenmp = '/openmp'
        clib_path = './' + extNameRoot + '/PlatformDependentCLibs/Windows'
    elif platform.system() == 'Linux':
        clib_path = './' + extNameRoot + '/PlatformDependentCLibs/Linux'
    else:
        print >> sys.stderr, "not fully supported platform, maybe rework the setup.py?"
        clib_path = None
    if clib_path:
        if os.path.isdir(clib_path):
            include_dirs = [clib_path, ]
        else:
            print "Path to CLibs "+repr(clib_path)+" does not exist. Check this if a C-Library/Header file is missing."

    extPath = extName.replace(".", os.path.sep) + ".pyx"
    dirname = os.path.dirname(extName.replace(".", os.path.sep) + ".pyx")
    # add .c files
    c_files = [os.path.join(dirname, f) for f in os.listdir(dirname) if
               os.path.isfile(os.path.join(dirname, f)) and f.endswith('.c')]
    sourcefiles = [extPath, ] + c_files
    return Extension(
        extName,
        sourcefiles,
        extra_compile_args=[fopenmp, ],
        extra_link_args=[fopenmp, ],
        include_dirs=include_dirs
    )


# get the list of extensions
extNames = scandir("Libs")
print extNames

# and build up the set of Extension objects
extensions = [makeExtension(name) for name in extNames]

# finally, we can pass all this to distutils
setup(
    ext_modules=cythonize(extensions, build_dir="build"),
)
