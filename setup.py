#!/usr/bin/env python

import shutil
import os

from distutils.core import setup
try: # for pip >= 10
    from pip._internal.req import parse_requirements
except ImportError: # for pip <= 9.0.3
    from pip.req import parse_requirements



setup(name='aida_env',
      version='1.0.1',
      description='Aida Learning Environment powered by GYM',
      author='Clement Jambou',
      author_email='dev@unsupervised.ai',
      url='https://github.com/unsupervisedai/aida_gym',
      packages=['aida_env']
      )
