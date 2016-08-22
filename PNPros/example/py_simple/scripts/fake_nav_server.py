#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Fri Jul 22 13:12:27 2016

@author: cd32
"""

import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from py_simple.msg import FakeNavAction, FakeNavResult
from pnp_msgs.msg import ActionResult


class NavServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=FakeNavAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("Done")

    def execute_cb(self, goal):
        rospy.loginfo("moving robot from '%s' to '%s'" % (goal.to, goal.from_))
        rospy.sleep(2.0)
        rospy.loginfo("robot at '%s'" % goal.to)
        res = FakeNavResult()
        res.result.append(ActionResult(cond="robot_at__"+goal.from_, truth_value=ActionResult.FALSE))
        res.result.append(ActionResult(cond="robot_at__"+goal.to, truth_value=ActionResult.TRUE))
        self._ps.set_succeeded(res)

if __name__ == "__main__":
    rospy.init_node("goto")
    n = NavServer(rospy.get_name())
    rospy.spin()

