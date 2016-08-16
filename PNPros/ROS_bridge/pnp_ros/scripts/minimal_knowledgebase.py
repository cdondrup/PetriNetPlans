#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:55:26 2016

@author: Christian Dondrup
"""

import rospy
from pnp_msgs.srv import PNPCondition, PNPConditionResponse, UpdateKnowledgeBase, UpdateKnowledgeBaseResponse
import yaml


class MinimalKnowledgebase(object):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        self.knowledgebase = {"hello": 1}
        kb_file = rospy.get_param("~kb_file", default="")
        if kb_file != "":
            for k, v in self.load_yaml(kb_file):
                self.knowledgebase[k] = v
        rospy.Service("PNPConditionEval", PNPCondition, self._condition_eval_cb)
        rospy.Service("~update_knowledgebase", UpdateKnowledgeBase, self._update_knowledgebase)
        rospy.loginfo("Done.")

    def load_yaml(self, f):
        with open(f, 'r') as y:
            return yaml.load(y)

    def _condition_eval_cb(self, req):
        rospy.loginfo("Evaluating '%s': %s" % (req.cond, str(self.knowledgebase[req.cond])))
        return PNPConditionResponse(self.knowledgebase[req.cond] if req.cond in self.knowledgebase else -1)

    def _update_knowledgebase(self, req):
        self.knowledgebase[req.cond] = req.truth_value
        print self.knowledgebase
        return UpdateKnowledgeBaseResponse()

if __name__ == "__main__":
    rospy.init_node("minimal_knowledgebase")
    MinimalKnowledgebase(rospy.get_name())
    rospy.spin()