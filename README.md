# CarND-Controls-MPC

## Overview
In this project model predictive control (MPC) technique is used to control steering and acceleration of a car in simulator. MPC requires model of controlled object (vehicle) and some information about environment (trajectory of track central line in front of the vehicle) to calculate control inputs.

## Vehicle model
Following class instructions classic bicycle model is extended by equations evaluating errors dynamics. This leads to model which describes following states: position - `x` and `y`, orientation - `psi`, speed - `v`, cross-track error `cte`, orientation error - `epsi`, which can be seen in lines 108-114 of MPC.cpp file.

There are two actuators: steering `delta` and acceleration `a`, which are used in model and which are generated during optimization process. So the aim is to minimize cost function

## Timestep parameters
Driving is quite dynamic process so there is no reason to consider long (in terms of time) prediction horizon. Due to same reasons it is good to use shot timesteps to be able to address fast changing in car's state. With this in mind initial values were set to `N = 50` and `dt = 0.01` which give 0.5 second of prediction horison however experiments showed that N is to high (may be only for my laptop) and dt is to short. Experiments led to final values `N = 10` and `dt = 0.05`.

## Reference trajectory (waypoints)
Algorithm receive trajectory of the track centre line in global coordinate frame. Transformation is used to map this trajectory in car's frame (main.cpp, lines 106-116). After that a polynomial is fitted to the trajectory and further used to calculate calculate control signals.

## Latency
It required to introduce special variable `lat_idx` to take latency into account. This variable uses lengths of timestep and latency to calculate (MPC.h, line 17) which control signal have to be send to car (main.cpp, lines 135-136). Also latency introduces constrains which require control signals affected by latency (falling to specific time interval) to be equal to previous control signals (MPC.cpp, lines 182-185 and 195-198).
