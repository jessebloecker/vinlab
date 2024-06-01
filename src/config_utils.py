#!/usr/bin/env python
import numpy as np
import array_utils
from geometry_utils import as_scipy_rotation
import yaml


class ConfigurationError(Exception):
    pass


# def check_keys(name,keys):
def check_keys(name,config):
    """
    Check if the keys/sub-keys provided in the yaml file are valid in their context.

    params:
        name: string, the name of the class or object that the keys are associated with
        config: dictionary whose keys (the sub-keys) will be checked according the entry in 'valid_keys.yaml' corresponding to 'name'.
    
    The following sets of keys are defined in 'valid_keys.yaml' for each name (class):
        
        required:            all of these keys must be included
        required_exclusive:  exactly one of these keys must be included (unless it contains any groups of codependent keys)
        optional:            any of these keys may optionally be included
        codependent:         if any of these keys are included, all of them must be included

    If the provided keys are valid, return the provided keys as a set, otherwise raise a ConfigurationError.
    """

    keys = set(config.keys())
    _list = yaml.load(open('/home/jesse/ros2_ws/src/vinlab/config/valid_keys.yaml'),Loader=yaml.FullLoader) #list of dictionaries
    valid = next(i for i in _list if i['name'] == name) #the the relevant dictionary corresponding to 'name'
    req = set(valid['required'])
    req_excl = set(valid['required_exclusive'])
    options = set(valid['optional'])
    codeps = [set(i) for i in valid['codependent']]
    allowed = req | req_excl | options

    codeps_req = [] #handle situation when codependent keys are in 'required_exclusive', meaning they are required as a group, if any are included 
    for codep in codeps:
        if len(codep&keys) > 0 and not codep <= keys: #then you provided some but not all of the codependent keys
            raise ConfigurationError('key(s): {} also requires: {} to be specified in configuration for \'{}\''
                                     .format(', '.join("'{0}'".format(i) for i in list(codep&keys)),
                                             ', '.join("'{0}'".format(i) for i in list(codep-keys)), name))
        codeps_req = codeps_req + [codep&req_excl] if codep <= req_excl else codeps #add the codep set to the list if it's a subset of 'required_exclusive'
      
    if not keys <= allowed: #then you provided a key that is not allowed
        raise ConfigurationError('unknown key(s): {} in configuration for \'{}\''
                                 .format(', '.join("'{0}'".format(i) for i in list(keys-allowed)),name))
    
    if not req <= keys: #then you did not provide all of the required keys
        raise ConfigurationError('required key(s): {} not found in configuration for \'{}\''
                                 .format(', '.join("'{0}'".format(i) for i in list(req-(keys&req))),name))
    
    if req_excl and len(keys&req_excl) != 1: #then you provided more than one key from 'required_exclusive'
        if keys&req_excl not in codeps_req: #then you did not provide a exactly one codependent group from 'required_exclusive'
            raise ConfigurationError('invalid combination of keys: {} in configuration for \'{}\''
                                     .format(', '.join("'{0}'".format(i) for i in list(keys&req_excl)),name))
        
    config_mode = keys&req_excl
    return config_mode, config


def config_transform(config):
    _from = config['from'] if 'from' in config.keys() else None
    rotation  = np.array(config['rotation']).astype(float)
    rot = as_scipy_rotation(rotation)
    pos = np.array(config['translation']).astype(float)
    return (rot,pos,_from)


