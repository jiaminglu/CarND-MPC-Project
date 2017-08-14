# CarND-Controls-MPC

## Video
[![Alt text](https://img.youtube.com/vi/t6vXwu8Kk0k/0.jpg)](https://www.youtube.com/watch?v=t6vXwu8Kk0k)

## Overview
In this project model predictive control (MPC) technique is used to control steering and acceleration of a car in simulator. MPC requires model of controlled object (vehicle) and some information about environment (trajectory of track central line in front of the vehicle) to calculate control inputs.

## Vehicle model
Following class instructions classic bicycle model is extended by equations evaluating errors dynamics. This leads to model which describes following states: position - `x` and `y`, orientation - `psi`, speed - `v`, cross-track error `cte`, orientation error - `epsi`, which can be represented by the following equations: 
```
x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
v_[t+1] = v[t] + a[t] * dt
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
This model is used to create constrains for optimizer in lines 108-114 of MPC.cpp file.

There are two actuators: steering `delta` and acceleration `a`, which are used in model and which are generated during optimization process.

## Timestep parameters
Driving is very dynamic process so there is no reason to consider long (in terms of time) prediction horizon. Due to same reasons it is good to use shot timesteps to be able to address fast changing in car's state. With this in mind initial values were set to `N = 50` and `dt = 0.01` which give 0.5 second of prediction horison however experiments showed that `N` is to high (may be only for my laptop) and `dt` is to short. Experiments led to final values `N = 10` and `dt = 0.05`.

## Waypoints
Algorithm receive waypoints in global coordinate frame. Geometric transformation is used to map waypoints in car's frame (main.cpp, lines 106-116), it includes shifting and rotation:
```
//trigonometric precalculation for optimization
double cos_psi = cos(psi);
double sin_psi = sin(psi);

//transformation
for (int i = 0; i < ptsx_tf.size(); i++) {
    double x = ptsx[i]-px;
    double y = ptsy[i]-py;
    ptsx_tf[i] = x * cos_psi + y * sin_psi;
    ptsy_tf[i] = -x * sin_psi + y * cos_psi;
}
```
After that a polynomial is fitted to the trajectory (main.cpp, line 119) and is further used to calculate control signals.

## Latency
It required to introduce special variable `lat_idx` to take latency into account. This variable uses lengths of timestep and latency to calculate (MPC.h, line 17) which control signal have to be send to car (main.cpp, lines 135-136). Also latency introduces constrains which require control signals affected by latency (falling to specific time interval) to be equal to previous control signals (MPC.cpp, lines 182-185 and 195-198).
