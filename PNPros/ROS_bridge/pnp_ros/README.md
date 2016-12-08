# PNP ROS package

This package uses the PNP and provides functionality to use ROS action servers as actions. This node either executes a plan that is saved in a specific location or it listens to the topic `/planToExec` where such a plan is published as an xml string.

## Running PNP ROS

Start the PNP execution system using the provided launch file:

```
roslaunch pnp_ros pnp_ros.launch
```

This has the following optional parameters:

_All these parameters can safely be ignored using the MuMMER system and are just mentioned for the sake of completeness._

* `plan_folder` _Default: pmp_ros/plans_: This specifies a path where to look for plans.
* `current_plan` _Default: stop_: This specifies the name of the plan to load from `plan_folder`. `stop` is a special keyword meaning that no plan is loaded. If started with a specific plan, execution will start immediately.
* `use_java_connection` _Default: false_: This specifies whether the java introspection tool can connect to the PNP node or not.

## Creating custom actions

The PNP executes actions based on their name. These actions are standard ROS ActionServers. These have to be registered with the PNP node via a service. Python wrappers are provided to hide this from the user. To use C++ action servers, you can either write a python wrapper that calls your C++ server or reimplement the registering functionality.

### Pyhton

The standard template for a PNP action is as follows

```python
import rospy
from pnp_plugin_server.pnp_simple_plugin_server import PNPSimplePluginServer
from my_package.msg import MyPNPAction, MyPNPResult
from pnp_msgs.msg import ActionResult


class TerminateInteraction(object):
    def __init__(self, name):
        rospy.loginfo("Starting %s ..." % name)
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=MyPNPAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
        rospy.loginfo("... done")
        
    def execute_cb(self, goal):
        # Do your computations here
        # ...
        ###

        # Creating the result
        res = MyPNPResult()
        res.result.append(ActionResult(cond="my_true_predicate", truth_value=True))
        res.result.append(ActionResult(cond="my_true_predicate", truth_value=False))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)


if __name__ == "__main__":
    rospy.init_node("terminate_interaction")
    TerminateInteraction(rospy.get_name())
    rospy.spin()
```

The main componants are the `PNPSimpleActionServer`:

```python
        self._ps = PNPSimplePluginServer(
            name=name,
            ActionSpec=MyPNPAction,
            execute_cb=self.execute_cb,
            auto_start=False
        )
        self._ps.start()
```

Which provides a thin wrapper around the `SimpleActionServer` and therefore all its functionality. In addition to that, it also takes care of registering your server with the PNP node. A `PNPPluginServer` is also available providing a wrapper for `ActionServer`. Both can be used the same way as the original ROS variants.

In order to understand the feedback system of the action, let's have a look at the action first:

```
string spam 
float32 eggs
bool foo
---
pnp_msgs/ActionResult[] result
---
```

As you can see, the goal can consist of any of the basic datatypes (no highlevel types like `geometry_msgs/Pose`, etc. are currently supported) and the result contains an array of `pnp_msgs/ActionResult`. These results are automatically interpreted based on your implementation of the `update_knowledgebase` function described below. So simply adding the predicates and their truth value as a result will add or remove them from the knowledge base:

```python
        res = MyPNPResult()
        res.result.append(ActionResult(cond="my_true_predicate", truth_value=True))
        res.result.append(ActionResult(cond="my_true_predicate", truth_value=False))
        if self._ps.is_preempt_requested():
            self._ps.set_preempted()
        else:
            self._ps.set_succeeded(res)
```


## Creating custom PNP bridges

PNP relies on an existing plan, knowledge base, and feedback mechanism. PNP itself is only responsible for the execution of the tasks and does not decide in which order they are excuted. To this, you will need to bridge PNP with a planning system of your choice. _For information on how to generate PNP plans from your planning system output, please refer to the README of the `pnpgen_ros` package._

### Using the knowledge base

PNP provides an abstract class which contains two methods that have to be overridden. The minimum working example is: (which can be found in `minimal_knowledgebase.py`)

```python
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
```

This node keeps its simple knowledge base in a dictionary and initialises it from a yaml file on start-up. The two functions that are are important here are the `quesry_knowledgebase`:

```python
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
```

Which should return `True` or `False` depending on if the given predicate is `True` or `False` in the KB and the `update_knowledgebase` function:

```python
    def update_knowledgebase(self, predicate, truth_value):
        """update the knowledge base.
        :param predicate: The condition as a string taken from the PNP.
        :param truth_value: (int) -1 (unknown), 0 (false), 1 (true)
        """
        self.knowledgebase[predicate] = truth_value
        print self.knowledgebase
```

which adds the given predicate to the KB using the `truth_value`. Currently, the predicate can only be a string but since everything can be serialised this shouldn't be a problem.

### Implementing planning feedback

The PNP has two possible outcomes: `success` or `failure`. These need to be reported to the planning system in order to decide if the plan was successful or if the planner has to replan using the updated knowledge base. A minimum example of this would be: (which can be found in `minimal_planner_feedback.py`)

```python
#!/usr/bin/python
# -*- coding: utf-8 -*-
"""
Created on Tue Aug 16 14:55:26 2016

@author: Christian Dondrup
"""

import rospy
from pnp_planning_system.pnp_planning_abstractclass import PNPPlanningAbstractclass


class MinimalPlannerFeedback(PNPPlanningAbstractclass):
    def __init__(self, name):
        rospy.loginfo("Starting '%s'." % name)
        super(MinimalPlannerFeedback, self).__init__()
        rospy.loginfo("Done.")

    def goal_state_reached(self):
        """Notify planning system that the goal has been reached
        """
        print "### HAPPY PANDA!"
        return

    def fail_state_reached(self):
        """Notify planning system that a failure has been reached
        """
        print "### SAD PANDA!"
        return

if __name__ == "__main__":
    rospy.init_node("minimal_planner_feedback")
    MinimalPlannerFeedback(rospy.get_name())
    rospy.spin()
```

where the two functions:

```python
    def goal_state_reached(self):
        """Notify planning system that the goal has been reached
        """
        print "### HAPPY PANDA!"
        return

    def fail_state_reached(self):
        """Notify planning system that a failure has been reached
        """
        print "### SAD PANDA!"
        return
```

Have to be implementd to send fedback to your planning node. They will be triggered when the PNP reaches a fail or goald state, repsectively.