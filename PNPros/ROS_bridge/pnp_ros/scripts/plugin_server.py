#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Wed Jul 20 13:28:19 2016

@author: Christian Dondrup
"""

import rospy
import rostopic
import roslib
from actionlib import ActionServer, ActionClient
from actionlib_msgs.msg import GoalStatus
from pnp_msgs.srv import PNPRegisterServer, PNPRegisterServerResponse
from pnp_msgs.srv import PNPUnregisterServer, PNPUnregisterServerResponse
from pnp_msgs.srv import PNPCondition, PNPConditionResponse
from pnp_msgs.msg import PNPAction, PNPFeedback, PNPResult
from threading import Thread


START = "start"
INTERRUPT = "interrupt"
END = "end"

GH = "goal_handle"
SN = "server_name"
G  = "goal"
GT = "goal_type"
S  = "server"


class PNPPluginServer(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s" % name)
        self.__servers = {}
        self.__goals = {}
        self.__conditions = {"hello": True}
        self._as = ActionServer(
            name, 
            PNPAction, 
            goal_cb=self._goal_cb, 
            cancel_cb=None, 
            auto_start=False
        )
        rospy.Service("~register_server", PNPRegisterServer, self.__register_callback)
        rospy.Service("~unregister_server", PNPUnregisterServer, self.__unregister_callback)
        rospy.Service('PNPConditionEval',
                  PNPCondition,
                  self._condition_eval_cb)
        self._as.start()
        rospy.loginfo("%s started" % name)
        
    def get_goal_type(self, action_name):
        topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
        # remove "Action" string from goal type
        assert("Action" in topic_type)
        return roslib.message.get_message_class(topic_type.replace('Action',''))
        
    def get_action_type(self, action_name):
        topic_type = rostopic._get_topic_type("/%s/goal" % action_name)[0]
        # remove "Goal" string from action type
        assert("Goal" in topic_type)
        return roslib.message.get_message_class(topic_type.replace('Goal',''))
        
    def _condition_eval_cb(self, req):
        print "+++++++++", req
        try:
            return PNPConditionResponse(self.__conditions[req.cond])
        except KeyError:
            return PNPConditionResponse(False)

    def __register_callback(self, req):
        name = req.action_name.replace('/','')
        rospy.loginfo("Registering '%s' action server" % name)
        if name not in self.__servers:
            self.__servers[name] = {
                S: ActionClient(name, self.get_action_type(name)),
                GT: self.get_goal_type(name)
            }
            rospy.loginfo("Waiting for '%s' action server to start." % name)
            self.__servers[name][S].wait_for_server()
            rospy.loginfo("'%s' action server started." % name)
            print self.__servers
            return PNPRegisterServerResponse(True)
        else:
            rospy.logwarn("'%s' already registered. Won't do anything." % name)
            return PNPRegisterServerResponse(False)
        
    def __unregister_callback(self, req):
        name = req.action_name.replace('/','')
        rospy.loginfo("Unregistering '%s' action server" % name)
        if name in self.__servers:
            del self.__servers[name]
            print self.__servers
            return PNPUnregisterServerResponse(True)
        else:
            rospy.logwarn("'%s' not registered. Won't do anything." % name)
            return PNPUnregisterServerResponse(False)
        
    def _goal_cb(self, gh):
        g = gh.get_goal()
        if g.function == START:
            gh.set_accepted()
            t = Thread(target=self._execute, args=(gh,))
            self.__goals[g.id] = {GH: gh, SN: g.name}
            print self.__goals
            t.start()
        elif g.function in (INTERRUPT, ): # END): END is being irgnored due to inconsistent IDs
            gh.set_accepted()
            self._cancel(gh)
        else:
            rospy.logwarn("Unknown function: '%s'." % g.function)
            gh.set_rejected(PNPResult(result='REJECTED'), 'REJECTED')
            
    def feedback_cb(self, gh, feedback, parent_gh):
        parent_gh.publish_feedback(feedback=PNPFeedback(feedback.__str__()))
#        print feedback
    
    def _cancel(self, gh):
        g = gh.get_goal()
        if g.id in self.__goals:
            if g.function == INTERRUPT:
                if gh.status_tracker.status.status in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                    try:                    
                        self.__goals[g.id][G].cancel()
                    except KeyError as e:
                        rospy.logerr(e)
                    self.__goals[g.id][GH].set_canceled(PNPResult(result='INTERRUPTED'), 'INTERRUPTED')
            else:
                self.__goals[g.id][GH].set_succeeded(PNPResult(result='OK'), 'OK')
            
            del self.__goals[g.id]
    
    def _execute(self, gh):
        g = gh.get_goal()
        rospy.loginfo("Executing: %s" % g)
        try:
            ng = self.__servers[self.__goals[g.id][SN]][GT]()
            for param, slot, t in zip(g.params.split('_'), ng.__slots__, ng._slot_types):
                setattr(ng, slot, type(getattr(ng,slot))(param))
            print ng

            self.__goals[g.id][G] = self.__servers[self.__goals[g.id][SN]][S].send_goal(ng, feedback_cb=lambda *x: self.feedback_cb(*x, parent_gh=gh))
            while self.__goals[g.id][G].get_goal_status() in (GoalStatus.ACTIVE, GoalStatus.PENDING):
                rospy.sleep(1.)
        except KeyError as e:
            rospy.logerr(e.message)
        gh.set_succeeded(PNPResult(result='OK'), 'OK')
        del self.__goals[g.id]


if __name__ == "__main__":
    rospy.init_node("pnp_plugin_server")
    p = PNPPluginServer("PNP")
    rospy.spin()
