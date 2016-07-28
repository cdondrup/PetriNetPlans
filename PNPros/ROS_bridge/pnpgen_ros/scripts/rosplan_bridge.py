#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 11:06:58 2016

@author: Christian Dondrup
"""

import rospy
from std_msgs.msg import String
from rosplan_dispatch_msgs.msg import CompletePlan
from pnp_msgs.srv import PNPGeneratePlan, PNPGeneratePlanRequest


class ROSPlanBridge(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self.pub = rospy.Publisher("/planToExec", String, queue_size=10, latch=False)
        rospy.sleep(0.5)
        rospy.Subscriber("/kcl_rosplan/plan", CompletePlan, self.callback)
        rospy.loginfo("Done.")
        
    def callback(self, msg):
        plan = ""
        for actions in msg.plan:
            plan += actions.name
            for param in actions.parameters:
                plan += "_%s" % param.value
            plan += ";"
        rospy.loginfo("Got plan '%s'" % plan[:-1])
        pnml = self.generate_pnml(plan[:-1])
        rospy.loginfo("Sending pnml")
        self.pub.publish(pnml)
        print "published"

    def generate_pnml(self, plan):
        s = rospy.ServiceProxy("/pnpgen_linear/generate_plan", PNPGeneratePlan)
        try:            
            s.wait_for_service(timeout=1.)
            return s(PNPGeneratePlanRequest(plan=plan)).pnml
        except rospy.ROSException:
            if not rospy.is_shutdown():
                rospy.logwarn("Unable to generate PNML, retrying.")          
                return self.generate_pnml(plan)
        
if __name__ == "__main__":
    rospy.init_node("rosplan_bridge")
    ROSPlanBridge(rospy.get_name())
    rospy.spin()
