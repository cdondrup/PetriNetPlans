#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:55:26 2016

@author: Christian Dondrup
"""

import rospy
from pnp_planning_system.pnp_planning_abstractclass import PNPPlanningAbstractclass


class MinimalPlannerFeedback(PNPPlanningAbstractclass):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        super(MinimalPlannerFeedback, self).__init__()
        rospy.loginfo("Done.")

    def goal_state_reached(self):
        """Notify planning system that the goal has been reached
        """
        print "### HAPPY PANDA!"
        return

    def fail_state_reached(self):
        """Notify planning system that a failure has been reached
        """
        print "### SAD PANDA!"
        return

if __name__ == "__main__":
    rospy.init_node("minimal_planner_feedback")
    MinimalPlannerFeedback(rospy.get_name())
    rospy.spin()