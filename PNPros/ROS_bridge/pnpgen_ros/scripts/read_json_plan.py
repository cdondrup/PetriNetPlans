#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 11:55:18 2016

@author: cd32
"""

import rospy
from std_msgs.msg import String
import json


def load_json(filename):
    with open(filename, 'r') as f:
        return json.dumps(json.load(f))

rospy.init_node("dummy_planner")
p = rospy.Publisher("/minimal_gen_bridge/plan", String, queue_size=1)
rospy.sleep(0.5)
p.publish(load_json(rospy.get_param("~plan")))
