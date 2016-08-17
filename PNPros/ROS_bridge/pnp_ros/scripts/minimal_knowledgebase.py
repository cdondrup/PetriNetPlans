#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:55:26 2016

@author: Christian Dondrup
"""

import rospy
from pnp_knowledgebase.pnp_knowledgebase_abstractclass import PNPKnowledgebaseAbstractclass
import yaml


class MinimalKnowledgebase(PNPKnowledgebaseAbstractclass):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        kb_file = rospy.get_param("~kb_file", default="")
        self.knowledgebase = self.load_yaml(kb_file) if kb_file != "" else {}
        super(MinimalKnowledgebase, self).__init__()
        rospy.loginfo("Done.")

    def load_yaml(self, f):
        with open(f, 'r') as y:
            return yaml.load(y)

    def query_knowledgbase(self, predicate):
        """querry the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :return (int) -1 (unknown), 0 (false), or 1 (true)
        """
        try:
            res = self.knowledgebase[predicate]
        except KeyError:
            res = -1
        finally:
            rospy.loginfo("Evaluating '%s': %s" % (predicate, str(res)))
            return res

    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        self.knowledgebase[predicate] = truth_value
        print self.knowledgebase

if __name__ == "__main__":
    rospy.init_node("minimal_knowledgebase")
    MinimalKnowledgebase(rospy.get_name())
    rospy.spin()