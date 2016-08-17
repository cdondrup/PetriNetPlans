# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 09:21:23 2016

@author: Christian Dondrup
"""

import rospy
from abc import ABCMeta, abstractmethod
from pnp_msgs.srv import PNPCondition, PNPConditionResponse, UpdateKnowledgeBase, UpdateKnowledgeBaseResponse
from pnp_msgs.msg import ActionResult


class PNPKnowledgebaseAbstractclass(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        rospy.Service("PNPConditionEval", PNPCondition, self._check_condition)
        rospy.Service("~update_knowledgebase", UpdateKnowledgeBase, self._update_knowledgebase)

    @abstractmethod
    def query_knowledgbase(self, predicate):
        """querry the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :return (int) -1 (unknown), 0 (false), or 1 (true)
        """
        return

    @abstractmethod
    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        return

    def _update_knowledgebase(self, req):
        self.update_knowledgebase(req.cond, req.truth_value)
        return UpdateKnowledgeBaseResponse()

    def _check_condition(self, req):
        rospy.loginfo("Checking condition: '%s'" % req.cond)
        res = self.query_knowledgbase(req.cond) if req.cond != "hello" else 1
        if not isinstance(res, int):
            raise TypeError("The result of 'querry_knowledgebase' has to be of type:", int)
        if not res in (ActionResult.UNKNOWN, ActionResult.FALSE, ActionResult.TRUE):
            raise TypeError("The result of 'querry_knowledgebase' has to be: -1 (unknown), 0 (false), or 1(true).")
        return PNPConditionResponse(res)
