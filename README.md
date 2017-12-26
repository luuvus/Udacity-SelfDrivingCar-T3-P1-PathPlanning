# Path Planning for Autonomous Driving
   
## Project Objectives
This project implement a path planning logic in C++ to safely navigate a automous vehicle around a virtual highway with other traffic that is driving +-10 MPH of the 50 MPH speed limit. The C++ application will receive the car's localization and sensor fusion data, along with a sparse map list of waypoints around the highway. The path planning will try to make the car go as close as possible to the 50 MPH speed limit and try to pass slower traffic when possible while account for the posibility of other cars will try to change lanes as well. The path planning logic will try to prevent the car from hitting other cars at all cost as well as driving inside of the marked road lanes at all times, unless going from one lane to another. The car should be able to make one complete loop around the 6946m highway. Since the car is trying to go 50 MPH, it should take a little over 5 minutes to complete 1 loop. Also the car should not experience total acceleration over 10 m/s^2 and jerk that is greater than 10 m/s^3.

Simulation Run Video: https://youtu.be/3DLJiJZJW9U

## Implementation Overview

This path planning application will generate 50 trajactory points (in Cartesian coordinate) that the simulator will use to make the car move along these X,Y points. The spacing of these points will determine how fast of the car will move.

#### Initial Implemenation
The initial implementation was to create points that evenly space out at 30m apart that cause the car to drive forward in given lane at more than 50 mile/hour, which has violated the speed limit and also these simple points produce frequent jerks during initial acceleration and when the new set of points are generated. Further more, this simple trajactory generation doesn't account for surrounding cars.

#### Frontal Collission Avoidance 
The application interate through the sensor fusion data that contain information about other cars and compare car's Frenet coordinate (s,d) position and with the target car's Frenet coordinate to calculate the other car's lane and spacing. At line 312, the application first check if the other car is in the same lane as the target car by compare other car's longitudinal value "d" to fall between the target car's lane width. If the other is in the same lane as the target car, then at line 316, application check if other car is in front of the target car and how far apart by compare the "s" values. If the spacing/gap between the two cars are less than 20m then the "too_close" flag is raised so that sub-sequence logics will decrease the velocity value for the target car to prevent collission with the car in front.

#### Lanes Change
The application aslo check for maneuverable conditions for any accessible lanes adjacent to the target car's lane during the interation of the sensor fusion data. At lines 323-357, the application use the other car's "d" to determine if the other car is in the left or right lanes. Then, it compares the target car's "s" value with other car's "s" value to calculate a reasonable opening space/gap in front and back. The "left_lane_open" and "right_lane_open" variables are set to true if there open space is safe for target car to move into. At lines 361-379, the application will make lane change decision when the target car's has encountered a "too_close" condition.

#### Path Smoothing
An open source "spline" library is used to calculate smooth trajactory points by create three reference anchor (x,y) points that evenly space out at 30m at lines 439-441. Once the anchor points are set to spline library at line 465, the spline can calculate trajactory points between the anchor points while account for curvature of the trajactory path. See lines 349-504 for details.

## Improvement
- Refactor the lanes change code into a function
- Create cost function to calculate the costs for maneuverable into any opening lanes


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./path_planning`.


## Other Information

#### The map of the highway is in data/highway_map.txt
Each waypoint in the list contains  [x,y,s,dx,dy] values. x and y are the waypoint's map coordinate position, the s value is the distance along the road to get to that waypoint in meters, the dx and dy values define the unit normal vector pointing outward of the highway loop.

The highway's waypoints loop around so the frenet s value, distance along the road, goes from 0 to 6945.554.

### Simulator.
You can download the Term3 Simulator which contains the Path Planning Project from the [releases tab (https://github.com/udacity/self-driving-car-sim/releases).

Here is the data provided from the Simulator to the C++ Program

#### Main car's localization Data (No Noise)

["x"] The car's x position in map coordinates

["y"] The car's y position in map coordinates

["s"] The car's s position in frenet coordinates

["d"] The car's d position in frenet coordinates

["yaw"] The car's yaw angle in the map

["speed"] The car's speed in MPH

#### Previous path data given to the Planner

//Note: Return the previous list but with processed points removed, can be a nice tool to show how far along
the path has processed since last time. 

["previous_path_x"] The previous list of x points previously given to the simulator

["previous_path_y"] The previous list of y points previously given to the simulator

#### Previous path's end s and d values 

["end_path_s"] The previous list's last point's frenet s value

["end_path_d"] The previous list's last point's frenet d value

#### Sensor Fusion Data, a list of all other car's attributes on the same side of the road. (No Noise)

["sensor_fusion"] A 2d vector of cars and then that car's [car's unique ID, car's x position in map coordinates, car's y position in map coordinates, car's x velocity in m/s, car's y velocity in m/s, car's s position in frenet coordinates, car's d position in frenet coordinates. 

## Details

1. The car uses a perfect controller and will visit every (x,y) point it recieves in the list every .02 seconds. The units for the (x,y) points are in meters and the spacing of the points determines the speed of the car. The vector going from a point to the next point in the list dictates the angle of the car. Acceleration both in the tangential and normal directions is measured along with the jerk, the rate of change of total Acceleration. The (x,y) point paths that the planner recieves should not have a total acceleration that goes over 10 m/s^2, also the jerk should not go over 50 m/s^3. (NOTE: As this is BETA, these requirements might change. Also currently jerk is over a .02 second interval, it would probably be better to average total acceleration over 1 second and measure jerk from that.

2. There will be some latency between the simulator running and the path planner returning a path, with optimized code usually its not very long maybe just 1-3 time steps. During this delay the simulator will continue using points that it was last given, because of this its a good idea to store the last points you have used so you can have a smooth transition. previous_path_x, and previous_path_y can be helpful for this transition since they show the last points given to the simulator controller with the processed points already removed. You would either return a path that extends this previous path or make sure to create a new path that has a smooth transition with this last path.

## Dependencies

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
    ```

---

## Tips

A really helpful resource for doing this project and creating smooth trajectories was using http://kluge.in-chemnitz.de/opensource/spline/, the spline function is in a single hearder file is really easy to use.    

