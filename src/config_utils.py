#!/usr/bin/env python
import numpy as np
import array_utils
from geometry_utils import as_scipy_rotation

#Functions for handling the yaml configuration files
def config_transform(config):
    rotation  = np.array(config['rotation']).astype(float)
    rot = as_scipy_rotation(rotation)
    pos = np.array(config['translation']).astype(float)
    return (rot,pos)