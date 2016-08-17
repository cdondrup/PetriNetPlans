#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 11:06:58 2016

@author: Christian Dondrup
"""

import rospy
from pnpgen_ros.pnpgen_bridge_abstractclass import PNPGenBridgeAbstractclass
from std_msgs.msg import String
import json


class MinimalBridge(PNPGenBridgeAbstractclass):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        super(MinimalBridge, self).__init__()
        rospy.Subscriber("~plan", String, self.callback)
        rospy.loginfo("Done.")

    def callback(self, msg):
        msg = json.loads(msg.data)
        self.generate_and_send_pnml(msg)
        rospy.loginfo("Sending pnml")

    def parse_plan_msg(self, msg):
        plan = self.new_plan()
        for step in msg["action_sequence"]:
            plan.actions.append(self.new_action_list(
                actions=[self.new_action(
                        name=str(action["name"]),
                        duration=int(action["duration"]),
                        parameters=[str(param) for param in action["parameters"]]
                    ) for action in step["concurrent_actions"]]
                )
            )
        plan.execution_rules = self.new_execution_rule_list(
            rules=[self.new_execution_rule(
                    timing=int(rule["timing"]),
                    action_name=str(rule["action"]),
                    condition=str(rule["condition"]),
                    recovery=self.new_action_list(actions=[self.new_action(
                        name=str(x["name"]),
                        duration=int(x["duration"]),
                        parameters=[str(y) for y in x["parameters"]]
                    ) for x in rule["recovery"]]
                )
            ) for rule in msg["execution_rules"]]
        )
        return plan

if __name__ == "__main__":
    rospy.init_node("minimal_gen_bridge")
    MinimalBridge(rospy.get_name())
    rospy.spin()
