#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 13:12:27 2016

@author: cd32
"""

import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from py_simple.msg import FakeSayAction, FakeSayResult
from pnp_msgs.msg import ActionResult


class SayServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=FakeSayAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("Done")

    def execute_cb(self, goal):
        rospy.loginfo("saying '%s' at '%s'" % (goal.text, goal.at))
        rospy.sleep(2.0)
        rospy.loginfo("said '%s'" % goal.text)
        res = FakeSayResult()
        res.result.append(ActionResult(cond="said_"+goal.text, truth_value=ActionResult.TRUE))
        self._ps.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("say")
    s = SayServer(rospy.get_name())
    rospy.spin()

