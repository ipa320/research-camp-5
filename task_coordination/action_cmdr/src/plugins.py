# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
# Copyright (C) 2011, Kyle Strabala
# Copyright (C) 2011-2012, Tim Niemueller [http://www.niemueller.de]
# Copyright (C) 2011, SRI International
# Copyright (C) 2011, Carnegie Mellon University
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""
Loads script server plugins.
Based on work done at CMU PRL and SRI for manipapp (TN).
"""

PKG = 'action_cmdr'
import roslib; roslib.load_manifest(PKG)

import os
import sys
import imp
import traceback
import inspect

import roslib.rospack
import roslib.packages
import rospy

from action_cmdr.abstract_action import AbstractAction

#from IPython.Shell import IPShellEmbed
#ipshell = IPShellEmbed()

def load_module(module_name, actions):
    """
    Introspect Python module and return actions defined in that file.
    @param module_name This is a universal name that is used in two ways:
    1. It is used as a ROS package name to load the packages manifest and add
    its src and lib sub-directories as Python search path
    2. It use used as Python module name to be loaded and searched for actions
    @return: a list of actions
    @rtype:  list of action class instances
    """
    pkg_dir = roslib.packages.get_pkg_dir(module_name)
    dirs = [os.path.join(pkg_dir, d) for d in ['src', 'lib']]
    sys.path = dirs + sys.path
    fp, pn, desc = imp.find_module(module_name)

    roslib.load_manifest(module_name)
    manip_actions_mod = imp.load_module(module_name, fp, pn, desc)

    #new_actions = [a[1](actions) #instantiates action
    #               # for all members of type class of the just opened module
    #               for a in inspect.getmembers(manip_actions_mod, inspect.isclass)
    #               # if the member is a (direct or indirect) sub-class of AbstractAction
    #               if issubclass(a[1], AbstractAction)
    #               # and it has an action name, i.e. it is not a abstract class
    #               and hasattr(a[1], "action_name") and a[1].action_name != None
    #               # and it has no disabled field or the field is set to false
    #               and (not hasattr(a[1], "disabled") or not a[1].disabled)]

    new_actions = []
    for a in inspect.getmembers(manip_actions_mod, inspect.isclass):
        if issubclass(a[1], AbstractAction) and hasattr(a[1], "action_name") and a[1].action_name != None and (not hasattr(a[1], "disabled") or not a[1].disabled):
            #print("Need to load %s" % a[1].action_name)
            if hasattr(a[1], "__init__") and len(inspect.getargspec(a[1].__init__).args) > 1:
                new_actions.append(a[1](actions))
            else:
                new_actions.append(a[1]())


    for a in new_actions:
        print a.action_name
        if a.action_name in actions:
            raise Exception("Action %s already exists" % a.action_name)

    actions.update(dict([(a.action_name, a) for a in new_actions]))
    return actions
