# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import actionlib
import utils as ut


class PNPSimplePluginServer(actionlib.SimpleActionServer):
    def __init__(self, name, ActionSpec, execute_cb=None, auto_start=True):
        self.ns = name
        actionlib.SimpleActionServer.__init__(self,
            name=name,
            ActionSpec=ActionSpec,
            execute_cb=execute_cb,
            auto_start=auto_start
        )
        
    def start(self):
        actionlib.SimpleActionServer.start(self)
        ut.register_plugin_client(self.ns)
        rospy.on_shutdown(lambda: ut.unregister_plugin_client(self.ns))
