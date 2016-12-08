# The ROS wrapper for PNP generation

PNP is only concerned with the execution of plans but not with their creation. In order to use PNP to turn your plan into a fast and robust petri-net, this package provides several functionalities.

## Creating a PNPgen bridge

This package provides an abstract python class `PNPGenBridgeAbstractclass` containing all the necessary functionality to create a valid Petri-Net Plan. In order to achieve this, your implementing class needs to override:

```python
    @abstractmethod
    def parse_plan_msg(self, msg):
        """Parse the incoming plan.
        :return pnpgen_ros/Plan
        """
        return
```

which receives an abstract message that comes out of your planning node and is then used to transform it into a `pnpgen_ros/Plan` message. To facilitate this process, the abstract class offers several convenience functions to create objects.

### Creating a new plan

Using the function

```python
new_plan(actions=None, execution_rules=None)
```

you can eaisly create a new and valid `pnpgen_ros/Plan` message that either be prefilled with actions and execution rules or just be empty to leave this to be added afterwards. If initial data is provided, `actions` has to be a `pnpgen_ros/PNPActionArray` and `execution_rules` has to be `pnpgen_ros/PNPExecutionRuleArray`, otherwise this is prepopulated as empty arrays of that type.

### Creating a new Action(Array)

Each plan can be comprised of a list of lists of actions. To elaborate, the simplest possibility is a list of atomic actions that are executed one after the other. The more likely case is that you would like to execute actions concurrently. To be able to allow for both, the action array could look like this:

```
actions = [a1, a2, [a3.1, a3.2, a3.3], a4]
```

where `a1` is executed first, `a2` second followed by `a3.1`, `a3.2`, and `a3.3` being executed concurrently at the same time, and the finally `a4` is excuted after all actions `a3.*` are finished. There are two ways to create the list `actions` which can easily be combined. Let's assume you careted your plan:

```python
plan = self.new_plan()
```

Now you need to fill this plan to make it do something sensible. We can add a new action like this:

```python
            plan.actions.append(
                self.new_action_list(
                    actions=self.new_action(
                        name="a1",
                        duration=5,
                        parameters=[1, "foo"]
                    )
                )
            )
```

which adds action `a1` to the action list. We could repeat the same for action `a2`. The parameters are the values that will be given to the goal of the action. The action file in the case of `a1` could, therefore, look like this:

```
int32 spam 
string eggs
---
pnp_msgs/ActionResult[] result
---
```

_Please have a look at the readme of `pnp_ros` for a description of the `pnp_msgs/ActionResult`_. The name of the two fields of the goal is irrelevant here, they will be filled in order. So the first parameter is assigned to `spam` and the second to `eggs`. If the types do not match, the pnp will throw and error at execution time.

In order to add the concurrent actions `a3.*` we just use a list of actions:

```python
            plan.actions.append(
                self.new_action_list(
                    actions=[self.new_action(name="a3.1"), self.new_action(name="a3.2"), self.new_action(name="a3.3")]
                )
            )
```

These actions do not have parameters and a duration of 0. Speaking about duration, the duration value denfines if the action should have a sepcific duration. If the duration is set, a concurrent wait task is started that will terminate the action after the given duration. If the task is over before the wait task is up, the wait task will wait till the duration is over. Hence, specifying a duration results in the task always running for the specified time regardless of if it was successful or not.