#!/usr/bin/python

import sys

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

sys.modules[__name__] = WrapModule(sys.modules[__name__], actions)

def init(module_names):
    global actions
    for m in module_names:
        #print("Module: %s" % m)
        load_module(m, actions)
