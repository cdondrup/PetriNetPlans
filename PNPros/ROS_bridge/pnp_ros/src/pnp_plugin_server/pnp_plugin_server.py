# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 09:42:29 2016

@author: Christian Dondrup
"""

import rospy
import actionlib
import utils as ut


class PNPPluginServer(actionlib.ActionServer):
    def __init__(self, ns, ActionSpec, goal_cb, cancel_cb=actionlib.nop_cb, auto_start=True):
        self.ns = ns
        actionlib.ActionServer.__init__(self,
            ns=ns,
            ActionSpec=ActionSpec,
            goal_cb=goal_cb,
            cancel_cb=cancel_cb,
            auto_start=auto_start
        )
        
    def start(self):
        actionlib.ActionServer.start(self)
        ut.register_plugin_client(self.ns)
        rospy.on_shutdown(lambda: ut.unregister_plugin_client(self.ns))
