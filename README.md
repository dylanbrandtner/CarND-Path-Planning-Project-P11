#  Path Planning Project

[//]: # (Image References)
[image1]: ./doc/full_nav3x.gif  "3x"
[image2]: ./doc/FSM.png "FSM"

## Project Introduction
In this project, I implemented a path planner in C++ to navigate through traffic around a 6946m highway loop. The planner takes in current trajectory information (ex. position, speed, yaw) and "sensor fusion" data, which is the same trajectory information for surrounding cars.  It then determines the best trajectory based on the current and predicted states of all cars by using a cost function, which is tuned to avoid accidents and travel at the maximum safe speed.  It then generates a smooth trajectory using an open source [spline implementation](http://kluge.in-chemnitz.de/opensource/spline/).  In the project simulator, the result looks like this (video is at 3x speed due to gif size limitations):

![alt text][image1]

The green line represents the current planned trajectory for the car sent from the path planner logic to the simulator.  The speed limit is 50mph, so to avoid exceeding this, the target speed is set to 47mph.  As you can see, the car navigates traffic until it can drive at it's target speed again. 

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

## Implementation

Below I will walk through how I met each rubric requirement, but first, I will describe my overall solution in 3 parts: Prediction, Planning, and Trajectory Generation.

### Prediction

On a highway, vehicle prediction can be vastly simplified if one basic assumption is taken: All other vehicles are traveling at a constant speed. In practice (and even in the simulator it turned out), this is not actually true, but for the purposes of this exercise, it was reasonable trade-off as long as certain safety precautions are taken in the behavior planning phase.  With this assumption, to generate predictions, we simply take the position and speed of all vehicles found by sensor fusion, and decide how many points ahead to predict.  I chose only 1 second ahead of the current point.  The position of that point was calculated by simply adding the velocity to the previous position.  Prediction data is generated in `main.cpp` between lines 310-336.

### Behavior planning

The behavior planning logic is mostly contained in the `Vehicle` class (implemented in `vehicle.h\cpp`) and it's interaction with the cost functions in `cost.cpp`.  The initial logic\structure here was copied from the behavior planning lesson materials, and then significantly modified to fit this project.  The `Vehicle` class contains the state of the vehicle, it's current kinematics (position, speed, acceleration) and functions to update the state and kinematics based on it's current lane.  It also maintains a "Finite State Machine" with the following states: 

![alt text][image2]

The Vehicle class uses the cost functions in `cost.cpp` to determine when/if it should move to another state.  There are 4 costs considered:

| Cost            | Weight | Description |
|:---------------:|:-------:|:--------------------------------------------------------------:|
| Lane Change     |  10&#x00B4 | If lane is different than current lane, cost is 1, otherwise 0 |
| Efficiency      |  10&#x00B5 | Difference between lane speed and target speed                 |
| Lane Congestion |  10&#x00B5 | Amount of vehicles within twice the vehicle "buffer" (25m)     |
| Lane Danger     |  10&#x00B8 | How close we are to vehicles within "danger buffer" (10m)      |

Once a state is chosen, it helps us determine the desired lane, and desired speed.  The desired speed is either the target speed (which is set to 47mph to avoid exceeding the speed limit), or the speed that will cause us to keep the desired "buffer" (25m) from the car ahead in that lane. The desired lane and speed is sent to the trajectory generation.  

### Trajectory generation

The final (and what ended up being the most complex) part of the problem was generating smooth trajectories that did not exceed the maximum jerk or acceleration limits.  In main.cpp, you can find the trajectory generation logic.  The project materials described generation of "Jerk Minimizing Trajectories", but this involved a significant amount of complex vector math.  Instead, the project materials suggested using an open source [spline implementation](http://kluge.in-chemnitz.de/opensource/spline/), which generates a "spline" between a given set of points.  The Project Q&A session described a single spline approach for x/y values, which worked well for a constant speed, but generating a smooth acceleration using this method proved to be _extremely_ difficult, as the desired acceleration is along the [frenet](https://medium.com/@kastsiukavets.alena/highway-path-planning-696215cbf062) "s" vector.  Thus, I setup two splines, one that mapped s vs. x vals, and one mapping s vs. y values.  Then I could purely use the frenet s values to generate the desired trajectory, and use the splines to calculate the x/y values expected by the simulator.  To change lanes, I simply modified the frenet d value by the desired amount in the initial splines.  Trajectory generation is implemented in `main.cpp` between lines 355-491.

## [Rubric Points](https://review.udacity.com/#!/rubrics/1020/view)

Here I will consider the rubric points individually.

### Compilation

#### Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.

##### Build Dependencies

* cmake >= 3.5
  * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
    
#### Simulator

You can download the Term3 Simulator which contains the Path Planning Project from [here](https://github.com/udacity/self-driving-car-sim/releases/tag/T3_v1.2).

### Valid Trajectories

#### The car is able to drive at least 4.32 miles without incident

#### The car drives according to the speed limit.

#### Max Acceleration and Jerk are not Exceeded.

#### Car does not have collisions.

#### The car stays in its lane, except for the time between changing lanes.

#### The car is able to change lanes


### Reflection