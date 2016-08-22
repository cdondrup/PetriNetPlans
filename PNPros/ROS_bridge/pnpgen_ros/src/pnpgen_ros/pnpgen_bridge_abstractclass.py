# -*- coding: utf-8 -*-
"""
Created on Thu Jul 28 11:06:58 2016

@author: Christian Dondrup
"""

from abc import ABCMeta, abstractmethod
import rospy
from std_msgs.msg import String
from pnpgen_ros.srv import PNPGeneratePlan, PNPGeneratePlanRequest
from pnpgen_ros.msg import Plan, PNPAction, PNPActionArray, PNPExecutionRule, PNPExecutionRuleArray


class PNPGenBridgeAbstractclass(object):
    __metaclass__ = ABCMeta

    BEFORE, DURING, AFTER = range(3)

    def __init__(self):
        self.pub = rospy.Publisher("/planToExec", String, queue_size=10, latch=True)
        rospy.sleep(0.1)

        self.restart_action = lambda:self.new_action("restart_action")
        self.start_action = lambda:self.new_action("start_action")
        self.skip_action = lambda:self.new_action("skip_action")
        self.restart_plan = lambda:self.new_action("restart_plan")
        self.fail_plan = lambda:self.new_action("fail_plan")

    @abstractmethod
    def parse_plan_msg(self, msg):
        """Parse the incoming plan.
        :return pnpgen_ros/Plan
        """
        return

    def new_plan(self, actions=None, execution_rules=None):
        if actions != None:
            if not isinstance(actions, list):
                raise TypeError("Field 'actions' has to be of type:", list)
            for a in actions:
                if not isinstance(actions, PNPActionArray):
                    raise TypeError("Field 'actions' has to contain entries of type:", PNPActionArray)
        else:
            actions = []
        if execution_rules != None:
            if not isinstance(execution_rules, PNPExecutionRuleArray):
                raise TypeError("Field 'execution_rules' has to be of type:", PNPExecutionRuleArray)
        else:
            execution_rules = PNPExecutionRuleArray()
        return Plan(
            actions=actions,
            execution_rules=execution_rules
        )

    def new_action_list(self, actions=None):
        if actions != None:
            if not isinstance(actions, list):
                actions = [actions]
            for a in actions:
                if not isinstance(a, PNPAction):
                    raise TypeError("'actions' has to be of type '%s' or '[%s]'" % (PNPAction,PNPAction))
        else:
            actions = []
        return PNPActionArray(
            pnp_action_array=actions
        )

    def new_action(self, name, duration=0, parameters=None):
        if not isinstance(name, str):
            raise TypeError("'name' has to be a string")
        if not isinstance(duration, int):
            raise TypeError("'duration' has to be a int")
        if parameters != None:
            if not isinstance(parameters, list):
                parameters = [parameters]
            for p in parameters:
                if not isinstance(p, str):
                    raise TypeError("'parameters' has to be a list of strings")
        else:
            parameters = []
        return PNPAction(
            name=name,
            duration=duration,
            parameters=parameters
        )

    def new_execution_rule_list(self, rules=None):
        if rules != None:
            if not isinstance(rules, list):
                rules = [rules]
            for a in rules:
                if not isinstance(a, PNPExecutionRule):
                    raise TypeError("'rules' has to be of type '%s' or '[%s]'" % (PNPExecutionRule,PNPExecutionRule))
        else:
            rules = []
        return PNPExecutionRuleArray(
            pnp_execution_rule_array=rules
        )

    def new_execution_rule(self, timing, action_name, condition, recovery):
        if not isinstance(timing, int):
            raise TypeError("'timing' has to be an int")
        if not isinstance(action_name, str):
            raise TypeError("'action_name' has to be a string")
        if not isinstance(condition, str):
            raise TypeError("'condition' has to be a string")
        if not isinstance(recovery, PNPActionArray):
            raise TypeError("'recovery' has to be a PNPActionArray")
        for r in recovery.pnp_action_array:
            if not isinstance(r, PNPAction):
                raise TypeError("'recovery' has to be a list of PNPAction")
        return PNPExecutionRule(
            timing=timing,
            action_name=action_name,
            condition=condition,
            recovery=recovery
        )

    def _check_msg_type(self, msg):
        if not isinstance(msg, Plan):
            raise TypeError("Message has to be of type:", Plan)
        if not isinstance(msg.actions, list):
            raise TypeError("Field 'actions' has to be of type:", list)
        for a in msg.actions:
            if not isinstance(a, PNPActionArray):
                raise TypeError("Field 'actions' has to contain entries of type:", PNPActionArray)
        if not isinstance(msg.execution_rules, PNPExecutionRuleArray):
            raise TypeError("Field 'execution_rules' has to be of type:", PNPExecutionRuleArray)

    def generate_and_send_pnml(self, msg):
        plan = self.parse_plan_msg(msg)
        self._check_msg_type(plan)
        s = rospy.ServiceProxy("/pnpgen/generate_plan", PNPGeneratePlan)
        while not rospy.is_shutdown():
            try:
                s.wait_for_service(timeout=1.)
                self.pub.publish(s(PNPGeneratePlanRequest(plan=plan)).pnml)
                return
            except rospy.ROSException:
                rospy.logwarn("Unable to generate PNML, retrying.")
