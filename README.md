# CarND-Controls-MPC

##Overview
In this project model predictive control (MPC) technique is used to control steering and acceleration of a car in simulator. MPC requires model of controlled object (vehicle) and some information about environment (trajectory of track central line in front of the vehicle) to calculate control inputs.

## Vehicle model
Following class instructions classic bicycle model is extended by equations evaluating errors dynamics. This leads to model which describes following states: `x, y, psi, v, cte, epsi`, which can be seen in lines 108-114 of MPC.cpp file.

## Timestep parameters
Driving is quite dynamic process so there is no reason to consider long (in terms of time) prediction horizon. Due to same reasons it is good to use shot timesteps to be able to address fast changing in car's state. With this in mind initial values were set to `N = 50` and `dt = 0.01` which give 0.5 second of prediction horison however experiments showed that N is to high (may be only for my laptop) and dt is to short. Experiments led to final values `N = 10` and `dt = 0.05`.

## Latency
It required to introduce special variable `lat_idx` to take latency into account. This variable uses lengths of timestep and latency to calculate (MPC.h, line 17) which control signal have to be send to car (main.cpp, lines 135-136). Also latency introduces constrains which require control signals affected by latency (falling to specific time interval) to be equal to previous control signals (MPC.cpp, lines 182-185 and 195-198).