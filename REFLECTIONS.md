# Model Predictive Control

Much like the [PID controller](https://github.com/liamondrop/pid-controls), Model Predictive Control (MPC) attempts to minimize the difference between an optimum trajectory and an agent's actual position. Model predictive controllers can exploit a dynamic model of a process, attempting to optimize the current timestep, while continuously predicting future timesteps up to some finite time horizon. This allows the MPC to anticipate future events and to take control actions accordingly. PID controllers do not have this predictive ability.

## The Model

For this project, we use the following kinematic model:

```
x[t+1]   = x[t] + v[t] * cos(psi[t]) * dt
y[t+1]   = y[t] + v[t] * sin(psi[t]) * dt
psi[t+1] = psi[t] + v[t]/Lf * delta[t] * dt
v[t+1]   = v[t] + a[t] * dt
```

These expressions describe what we expect the future position, orientation and speed of our vehicle to be in the future, given their current values.

- `t` is the current timestep and t+1 is the predicted next timestep
- `dt` is the difference between timesteps
- `x` and `y` are the coordinates of the vehicle
- `psi` is the vehicle orientation
- `v` is the velocity
- `Lf` is the distance between the front of the vehicle and its center of gravity
- `delta` is the steering angle

Additionally, we want to model the expected error (the difference between the ideal trajectory and the expected trajectory given the dynamic forces acting on our model) so that we can minimize it. In particular, we are interested in the distance of the vehicle from the trajectory, which we call the cross track error (cte), as well as the difference in the vehicle's orientation and the trajectory orientation. These errors are described by the following equations:

```
cte[t+1]     = f(x[t]) - y[t] + (v[t] * sin(psi_err[t]) * dt)
psi_err[t+1] = psi[t] - desired_psi[t] + (v[t]/Lf * delta[t] * dt)
```
The following terms are new:

- `f(x)` is the `y` value of the ideal trajectory at a given `x`, where `f` calculates a polynomial that fits the waypoints of the trajectory. Subtracting `y[t]` from this gives us the current cross track error
- `psi_err` is the orientation error
- `desired_psi` is the tangential angle of the polynomial `f` evaluated at `x[t]`, which can be calculated as `arctan(f'(x[t]))`. `f'(x)` is the derivative of the polynomial. Calculating this value gives us the desired orientation if the vehicle followed the ideal trajectory. Therefore, subtracting `desired_psi` from `psi` gives us the orientation error.

## Timestep Length and Elapsed Duration

Since we're dealing with a predictive model, we need to consider how far in the future to predict. That time horizon is defined as the product of a discreet number of steps (N) and the amount of time (dt) between each control input. Ideally, you would look as far forward as possible in order to anticipate future moves and make many small actuations along the way so that you can quickly correct for unexpected error. However, the longer the time horizon, the more computationally expensive the model becomes.

For this project, we are only given a small portion of the path the car is meant to follow around the track at any given time. After some experimentation, using a time horizon of 15 steps into the future with a time difference of 0.04 seconds between steps provided enough lookahead and small enough time between actuations to allow the car to drive safely at fairly high speeds (my max speed as of this writing is ~90mph).

## Polynomial Fitting and MPC Preprocessing

In main.cpp lines 111-123, I transformed the given waypoints to the vehicle's local coordinate system and fit a second degree polynomial to give a good approximation of the expected trajectory. Initially, I started by fitting a third degree polynomial to the waypoints, but as I increased the speed of the vehicle, this sometimes resulted erratic calculations, occasionally throwing the vehicle off the road in very dramatic fashion. Using a second degree polynomial still provided a good approximation of the trajectory, as we are only looking 0.6 seconds ahead and the path is never so complex as to need a higher order polynomial. This proved far more stable, enabling me to turn up the velocity without a lot of additional tweaking to the model.

## Model Predictive Control with Latency

In a real vehicle, there is typically a delay between the time we issue control outputs and when the car actually responds. This is simulated by adding a 100ms sleep between actuations. In order to deal with this latency, I augmented the predicted state at time t by predicting the position and orientation of the car an additional 100ms into the future. You can see this implemented in main.cpp on lines 139-146.
