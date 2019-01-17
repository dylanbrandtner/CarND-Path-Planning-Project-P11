#  Path Planning Project

[//]: # (Image References)
[image1]: ./doc/full_nav3x.gif  "3x"
[image2]: ./doc/full_nav3x.gif  "3x"

## Project Introduction
In this project, I implemented a path planner in C++ to navigate through traffic around a 6946m highway loop. The planner takes in current trajectory information (ex. position, speed, yaw) and "sensor fusion" data, which is the same trajectory information for surrounding cars.  It then determines the best trajectory based on the current and predicted states of all cars by using a cost function, which is tuned to avoid accidents and travel at the maximum safe speed.  It then generates a smooth trajectory using an open source [spline implementation](http://kluge.in-chemnitz.de/opensource/spline/).  In the project simulator, the result looks like this (video is at 3x speed due to gif size limitations):

![alt text][image1]

The green line represents the current planned trajectory for the car sent from the path planner logic to the simulator.  The speed limit is 50mph, so to avoid exceeding this, the target speed is set to 47mph.  As you can see, the car navigates traffic until it can drive at it's target speed again. 

### Goals
In this project your goal is to safely navigate around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. You will be provided the car's localization and sensor fusion data, there is also a sparse map list of waypoints around the highway. The car should try to go as close as possible to the 50 MPH speed limit, which means passing slower traffic when possible, note that other cars will try to change lanes too. The car should avoid hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.


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


### Reflection