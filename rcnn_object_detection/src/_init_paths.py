# --------------------------------------------------------
# Fast R-CNN
# Copyright (c) 2015 Microsoft
# Licensed under The MIT License [see LICENSE for details]
# Written by Ross Girshick
# --------------------------------------------------------

"""Set up paths for Fast R-CNN."""

import os.path as osp
import sys

def add_path(path):
    if path not in sys.path:
        sys.path.insert(0, path)

this_dir = osp.dirname(__file__)

# Add caffe to PYTHONPATH
caffe_path = osp.join(this_dir, '/home/nishant/py-faster-rcnn/', 'caffe-fast-rcnn', 'python')
add_path(caffe_path)

# Add lib to PYTHONPATH
lib_path = osp.join(this_dir, '/home/nishant/py-faster-rcnn/', 'lib')
add_path(lib_path)

# Add py_rcnn_folder PYTHONPATH
#rcnn_path = osp.join(this_dir, '/home/nishant/apc_ws/src/', '')
#add_path(rcnn_path)
