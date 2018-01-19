# CarND-Controls-MPC P10
Self-Driving Car Engineer Nanodegree Program

---

## The Model

Model Predictive Control (MPC) drives the car around the track with additional latency 100ms between actuator commands. The NMPC allows the car to follow the trajectory along a line (path), provided by the simulator in the World (map) coordinate space, by using predicted(calculated) actuators like steering angle and acceleration (throttle/brake combined).

The vehicle model is implemented is a kinematic bicycle model that ignore tire forces, gravity, and mass. This simplification reduces the accuracy of the models, but it also makes them more tractable. At low and moderate speeds, kinematic models often approximate the actual vehicle dynamics. Kinematic model is implemented using the following equations:
```
      x_[t+1] = x[t] + v[t] * cos(psi[t]) * dt
      y_[t+1] = y[t] + v[t] * sin(psi[t]) * dt
      psi_[t+1] = psi[t] + v[t] / Lf * delta[t] * dt
      v_[t+1] = v[t] + a[t] * dt
      cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt
      epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt
```
where   
* `(x,y)` is position of the vehicle; 
* `psi` is orientation of the vehicle; 
* `v` is velocity;
* `delta` and `a` are actuators like steering angle and aceleration (throttle/brake combined);
* `Lf` measures the distance between the front of the vehicle and its center of gravity (the larger the vehicle, the slower the turn rate); 
* `cte` is  cross-track error (the difference between the line and the current vehicle position y in the coordinate space of the vehicle); 
* `epsi` is the orientation error. The vehicle model is implemented in the `FG_eval` class.

Using the above set of model equations we can provide the system with actuator values to control and change the state of the vehicle.

### the selection of N and dt 
After trying different values, as shown in the below table,

|   N     |   dt     |
| ------- | -------- |
|    25   |   0.01   |
|    10   |   0.03   |
|    10   |   0.1    |

The number of points `N` and the time interval `dt` determine the prediction horizon.
The number of points impacts the controller performance as well.
I tried to keep the horizon around the same time the waypoints were on the simulator. 
With too many points the controller starts to run slower, 
and some times it went wild very easily. After these trials, 
I finally picked the last combination which contributes a smoother drive.


### Polynomial Fitting and MPC Preprocessing
Before fitting the polynomials, there is preprocessing that is done on the provided waypoints. Since the waypoints are in map perspective, they are transformed to vehicle perspective using the approach described [here](https://discussions.udacity.com/t/waypoints-going-crazy/270597/2).

```cpp
for (int i = 0; i < ptsx.size(); ++i) {
  waypoints_x_vals.push_back(cos(-psi) * (ptsx[i] - px)
                        - sin(-psi) * (ptsy[i] - py));
  waypoints_y_vals.push_back(sin(-psi) * (ptsx[i] - px)
                        + cos(-psi) * (ptsy[i] - py));
}
```

Following this, a third order polynomial is fitted (`polyfit` function) through the transformed waypoints and the `coeffs` obtained are used to predict the error in actual and reference trajectories. The initial state vector is then constructed and used for MPC processing.

### Model Predictive Control with Latency
To deal with latency in applying actuator controls to the vehicle (100ms), I used the kinematic model equations to predict the state after the latency period.

```cpp
double delta = j[1]["steering_angle"];
double throttle = j[1]["throttle"];

delta *= -1;
px = v * cos(delta) * latency;
py = v * sin(delta) * latency;
cte += v * sin(epsi) * latency;
epsi += v * delta * latency / Lf;
psi = v * delta * latency / Lf;
v += throttle * latency;
```

This new state is then sent to the MPC controller for getting the actuator values. This makes the vehicle handle latency correctly.


---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.)
4.  Tips for setting up your environment are available [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)
5. **VM Latency:** Some students have reported differences in behavior using VM's ostensibly a result of latency.  Please let us know if issues arise as a result of a VM environment.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).
