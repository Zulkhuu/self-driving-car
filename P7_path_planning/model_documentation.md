I followed same approach as Project Q&A video.

## Prediction
To keep track of vehicle prediction data easy to understand as possible, I created [Lane](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.h#L28) struct in [path_planner.h](src/path_planner.h) file. Since I only need closest car in front of me or back, Lane struct keep track of one vehicle for each direction. For example: Let's say there is a car in front of me in 30meters and another one in front of that car in further 40meters all in same lane.  In this case I only care about the car right in front of me(30 meters) and disregard the other car. Same logic applies for checking cars trailing behind me. 

When I want to check if side lane is free to switch, I check what is the distance(in Frenet s coordinate) of the closest cars in that lane. If the closest cars in front and behind me in that lane is farther than safe distance(parameter to tune), then I assume lane is safe to switch into. These checks are done in [UpdateLaneInfo](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.cpp#L30) function in [path_planner.cpp](src/path_planner.cpp) file.

## Behavior planning
I considered following 4 states for driving: 
 - KEEP_LANE : There is no vehicle in front of my current lane in close distance. Car should stay in the lane and keep maximum allowed speed as possible.
 - PREPARE_LANE_CHANGE : There is a vehicle in front of me. I should look for lane change if possible meanwhile keeping same speed with the car in front of me.
 - CHANGE_LANE_LEFT : Change to left lane as fast as possible.
 - CHANGE_LANE_RIGHT : Change to right lane as fast as possible.

State transition is then decided based on lane traffic information. Logic is fairly simple: 
 - If there is no car in front of me that is too close, drive as fast as possible in current lane. 
 - If there is a car in front of me that is too close, check if side lanes are free to switch
    - If one of the lane is free to switch, switch to that lane
    - If both of the lane is free to switch, switch to the one with least traffic in front.
    - If none of the lane is free to switch, follow the car in front with same speed and wait for other lanes to become free

State transition logic is done in UpdatePath() function in [Line176-232](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.cpp#L176) in [path_planner.cpp](src/path_planner.cpp) file.

## Trajectory Generation
Trajectory Generation closely follows the recommended approach in Project Q&A video. Instead of Polynomial Trajectory Generation, I opted to use awesome Spline library from [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/) as it was easy to use and recommended. 

Trajectory Generation consists of following steps:
 - First, 5 sparse points(anchor points) are selected for creating spline: 2 from previous path's tail end, 3 for further down the road in 30m, 60m and 90m. Combining previous path's end points allow new path to smoothly merge with the old one. This avoids unnecessary acceleration/jerk resulted from joinging two different trajectories. Implemented in [CalcAnchorPoints](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.cpp#L103) in [path_planner.cpp](src/path_planner.cpp) file.
 - Second, spline is created from previously calculated 5 anchor points(converted to car's local coordinate) using Spline library from [http://kluge.in-chemnitz.de/opensource/spline/](http://kluge.in-chemnitz.de/opensource/spline/). Implemented in UpdatePath() function's [Line238-239](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.cpp#L238) in [path_planner.cpp](src/path_planner.cpp) file.
 - If there are points from previous path that remains it is added in the beginning of the new path. It is implemented in UpdatePath() function's [Line244-248](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.cpp#L244)
 - Remaining points are resampled from previously created spline function. Distance between sample depends on velocity we want to achieve and spline function shape. It is implemented in UpdatePath() function's [Line250-273](https://github.com/Zulkhuu/self-driving-car/blob/efb25af32d534c52ff5f265b654fee2b335393db/P7_path_planning/src/path_planner.cpp#L250)

### Reflection
Currently I set following parameters in [path_planning.h](src/path_planner.h) for tuning trajectory generation. 
 - MAX_SPEED  = 49.5 MPH(22.13 m/s)
 - MAX_ACCELERATION = 8 m/s^2
 - FRONT_CLEARANCE_DIST = 40m
 - BACK_CLEARANCE_DIST = 15m

With above settings, I was able to create smooth path. However car seems to be overly cautious about assuming whether lane is free to switch most of the time. But if I set the parameters to be more aggressive(less distance for clearing, more acceleration), it results in collision or violation in some cases. Therefore, some improvement could be added by understanding other cars' dynamics and their future positions beter and better approximation when sampling spline function. 