# -*- coding: utf-8 -*-
"""
Created on Wed Aug 17 09:21:23 2016

@author: Christian Dondrup
"""

import rospy
from abc import ABCMeta, abstractmethod
from pnp_msgs.srv import PNPSuccess, PNPSuccessResponse, PNPFailure, PNPFailureResponse


class PNPPlanningAbstractclass(object):
    __metaclass__ = ABCMeta

    def __init__(self):
        rospy.Service("~success", PNPSuccess, self.__success)
        rospy.Service("~failure", PNPFailure, self.__failure)

    @abstractmethod
    def goal_state_reached(self):
        """Notify planning system that the goal has been reached
        """
        return

    @abstractmethod
    def fail_state_reached(self):
        """Notify planning system that a failure has been reached
        """
        return

    def __failure(self, req):
        self.fail_state_reached()
        return PNPFailureResponse()

    def __success(self, req):
        self.goal_state_reached()
        return PNPSuccessResponse()
