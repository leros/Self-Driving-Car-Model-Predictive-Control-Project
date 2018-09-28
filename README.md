[![Udacity - Self-Driving Car NanoDegree](https://s3.amazonaws.com/udacity-sdc/github/shield-carnd.svg)](http://www.udacity.com/drive)


### PART 2: Sensor Fusion, Localization, and Control
### Project 5: Model Predictive Control

In this project I implement Model Predictive Control to drive the car around the track. This time however the cross track error is not given, I'll have to calculate that myself! Additionally, there's a 100 millisecond latency between actuations commands on top of the connection latency.

---

## The Model
MPC is a method of process control that can represent the behavior of a complex dynamical system such as vehicles. It allows the current timeslot to be optimized and keeps future timeslots in account. The model optimizes a finite time-horizon, but only implement the current timeslot and then optimize again and again. Some critical components of the model:

- State of the vehicle: global position(x and y), orientation,  and current velocity
- Costs: cte and epsi
- Actuators: steer angle and throttle
- Update equations:
```
       x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
       y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
       psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
       v_[t+1] = v[t] + a[t] * dt
       cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
       epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```



## Timestep Length(N) and Elapsed Duration(dt)

The prediction horizon, or T, is the duration over which future predictions are made. T is the product of N and dt, where N is the number of timesteps to be predicated, and dt is how much time elapses between actuations. For example, if N were 10 and dt were 0.1, then T would be 1, i.e. 10 * 0.1. 

Some guidelines and tradeoffs for choosing the parameters:

- T should be as large as possible. However, the environment will eventually change enough that it won't make sense to predict too further into the future. Obviously, the speed of the vehicle has a great impact of the "future".
- dt should be as small as possible. A larger dt would result in less frequent actuation, which makes it harder to accurately approximate a continuous reference trajectory when the vehicle is turning round.
- Since N determine the number of variables optimized by the MPC, it dominates the computational cost. 

The final parameters I used are N = 10, dt = 0.10 and ref_v = 80, which means the car could drive at a speed up to 80 mph in the simulation. I tried the combination of N = 12 and dt = 0.12,  which didn't give good results when the car drove as fast as 80 mph.

## Polynomial Fitting and MPC Preprocessing
- I used a third-degree polynomial function to fit the waypoints to create a reference trajectory.

- I transformed the car reference angle to 90 degrees, so the car always points to the right, which simplified the computation afterward.


## Model Predictive Control with Latency
I incorporated the latency into the model by using the vehicle model starting from the current statue for the duration of the latency. 

``` C++
// predict state in 100ms
double latency = 0.1;
const double Lf = 2.67;
double steer_value = j[1]["steering_angle"];
double throttle_value = j[1]["throttle"];
double new_x, new_psi, new_v;
new_x = latency * v;
new_psi = -v*steer_value/Lf*latency;
new_v = v + throttle_value*latency;

state << new_x, 0, new_psi, new_v, cte, epsi;
```

In this way, MPC could deal with latency by explicity taking it into account.



