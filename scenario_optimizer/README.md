# ROS Scenario Optimizer

This ROS package `scenario optimizer` can be used to simulate a scenario with different parameters to optimize the cost function (distance between Scoomatic and Walker).\
The bayesian optimization used in this packaged is based on the implementation of [Fernando Noguiera](https://github.com/fmfn/BayesianOptimization) and is converted to be executable with Python 2.

## Starting the node

In order to use this node you need to first start the Carla Simulator and the complete navigation stack, including the scoomatic controller:
```
./CarlaUE4.sh
```
```
roslaunch scoomatic_scenario scoomatic_stack_and_scenario.launch
```

After that you can use the given launch file of this package:
```
roslaunch scenario_optimizer scenario_optimizer.launch
```

---

## Using the node

After launching the node it will execute the optimization of a given scenario with the parameters specified in the code. The optimized parameters in this case are the walking direction of the walker and the cost function is the negative distance between the vehicle and the walker.