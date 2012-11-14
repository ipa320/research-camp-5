#!/usr/bin/python

import sys
import roslib

from plugins import load_module

# required to allow dot notation access to dictionary
class DictObj(dict):
    __getattr__= dict.__getitem__
    __setattr__= dict.__setitem__
    __delattr__= dict.__delitem__
    
class WrapModule(object):
    def __init__(self, wrapped, actions):
        self.wrapped = wrapped
	self.actions = actions

    def __getattr__(self, name):
        if hasattr(self.wrapped, name):
            return getattr(self.wrapped, name)
        else:
            return getattr(self.actions, name)


actions = DictObj()
modules = []

sys.modules[__name__] = WrapModule(sys.modules[__name__], actions)

def init(package_name):
    #print("Initializing %s" % package_name)
    to_check = roslib.rospack.rospack_depends_1(package_name)

    action_modules = []

    for pkg in to_check:
        manifest_file = roslib.manifest.manifest_file(pkg, True)
        manifest      = roslib.manifest.parse_file(manifest_file)

        action_module_names = manifest.get_export('actions', 'action-modules')

        if action_module_names:
            for a in action_module_names:
                action_modules += a.split(',')

    #print("Action modules: %s" % action_modules)
    for a in action_modules:
        # Load that package's namespace
        roslib.load_manifest(a)

    load(action_modules)

def load(module_names):
    global actions
    global modules
    for m in module_names:
        if not m in modules:
            modules.append(m)
            load_module(m, actions)
