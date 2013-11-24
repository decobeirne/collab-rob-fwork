# Run "setup.py install" or "setup.py --help"
from distutils.core import setup, Extension

imageUtils_mod = Extension('imageUtils', sources = ['imageUtils.c'])

setup(name = "imageUtils",
      version = "1.0",
      description = "A module providing functionality to process image data for the purpose of training an object detection model.",
      ext_modules = [imageUtils_mod])
