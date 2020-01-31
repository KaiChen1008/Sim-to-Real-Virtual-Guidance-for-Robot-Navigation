#!/bin/bash

# need to install virtualenv and check your path to python3
virtualenv -p /usr/bin/python3 rlenv
# get into virtualenv
. rlenv/bin/activate
# need to install ros first
pip install pyyaml
pip install rospkg
pip install opencv-python
# pip install --upgrade tensorflow
pip install tensorflow==1.13.1



