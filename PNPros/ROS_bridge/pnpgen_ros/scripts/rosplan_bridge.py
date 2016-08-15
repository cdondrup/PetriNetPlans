#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 11:06:58 2016

@author: Christian Dondrup
"""

import rospy
from pnpgen_ros.pnp_bridge_abstractclass import PNPBridgeAbstractclass
from rosplan_dispatch_msgs.msg import CompletePlan


class ROSPlanBridge(PNPBridgeAbstractclass):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        super(ROSPlanBridge, self).__init__("/kcl_rosplan/plan")
        rospy.Subscriber("/kcl_rosplan/plan", CompletePlan, self.callback)
        rospy.loginfo("Done.")

    def callback(self, msg):
        self.generate_and_send_pnml(msg)
        rospy.loginfo("Sending pnml")

    def parse_plan_msg(self, msg):
        plan = self.new_plan()
        for action in msg.plan:
            plan.actions.append(
                self.new_action_list(
                    actions=[self.new_action(
                        name=action.name,
                        duration=int(action.duration),
                        parameters=[str(param.value) for param in action.parameters]
                    )]
#                    ,self.new_action(
#                        name=action.name,
#                        duration=2,
#                        parameters=["spam"]
#                    ), self.new_action(
#                        name=action.name,
#                        duration=4,
#                        parameters=["eggs"]
#                    )]
                )
            )
        plan.execution_rules = self.new_execution_rule_list(
            rules=[self.new_execution_rule(
                timing=self.BEFORE,
                action_name="goto",
                condition="(not spam)",
                recovery=self.new_action_list(actions=[self.new_action(
                    name="goto",
                    duration=2,
                    parameters="foo"
                ),
                self.new_action(
                    name="goto",
                    duration=5,
                    parameters="bar"
                ), self.skip_action()
                ])
            ),
            self.new_execution_rule(
                timing=self.AFTER,
                action_name="goto",
                condition="spam",
                recovery=self.new_action_list(actions=[self.new_action(
                    name="goto",
                    duration=10,
                    parameters="bar"
                ),self.restart_action()
                ])
            ),
            self.new_execution_rule(
                timing=self.DURING,
                action_name="goto",
                condition="action_failed",
                recovery=self.new_action_list(actions=[self.new_action(
                    name="goto",
                    duration=0,
                    parameters="bar"
                ),self.restart_action()
                ])
            )
            ]
        )
        return plan

if __name__ == "__main__":
    rospy.init_node("rosplan_bridge")
    ROSPlanBridge(rospy.get_name())
    rospy.spin()
