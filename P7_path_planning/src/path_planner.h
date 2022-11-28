#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <iostream>
#include <cmath>
#include <vector>
#include "json.hpp"
#include "helpers.h"

using nlohmann::json;
using std::string;
using std::vector;

#define MILES_TO_KM 1.609344
#define MPH_TO_MS MILES_TO_KM*1000.0/3600.0

#define LEFT_LANE 0
#define CENTER_LANE 1
#define RIGHT_LANE 2

enum NextState {   
    KEEP_LANE = 0, 
    PREPARE_CHANGE_LANE = 1, 
    CHANGE_LANE_RIGHT = 2,
    CHANGE_LANE_LEFT = 3
};

struct Lane{
    bool is_free;
    
    bool car_in_front;
    double front_car_distance;
    double front_car_speed;
    
    bool car_in_back;
    double back_car_distance;
    double back_car_speed;
};

struct CarState{
    // Cartesian coordinates j& speed
    double x;
    double y;
    double yaw;
    double speed;

    // Frenet coordinates
    double s;
    double d;

    // Reference pose for calculating local coordinate
    double ref_x;
    double ref_y;
    double ref_s;
    double ref_yaw;

    // Previous path data given to the Planner
    json previous_path_x;
    json previous_path_y;

    // Previous path's end s and d values 
    double end_path_s;
    double end_path_d;
};

class PathPlanner {

public:

    PathPlanner();
    virtual ~PathPlanner() {};

    // Update next path
    void UpdatePath(json j);
   
    // Load map waypoints info
    void SetMap(vector<double>& x, vector<double>& y, vector<double>& s, vector<double>& dx, vector<double>& dy);
 
    vector<double> next_x_vals;
    vector<double> next_y_vals;

private:

    // Number of waypoints to consider in path planning
    static constexpr int N_PATH_POINTS = 50;

    // One time step for simulation is 20ms
    static constexpr double TIME_STEP = 0.02;

    // Number of lanes
    static constexpr int N_LANES = 3;

    // Lane width in meter
    static constexpr double LANE_WIDTH = 4.0;

    // Max speed in meter/second
    static constexpr double MAX_SPEED = 49.5 * MPH_TO_MS;

    // Max accelration in meter/second^2
    static constexpr double MAX_ACCELERATION = 8;

    // Max velocity change in one step(20ms)
    static constexpr double MAX_VEL_CHANGE_PER_STEP = MAX_ACCELERATION * TIME_STEP;

    // Safe distance to the front car in meter
    static constexpr double FRONT_CLEARANCE_DIST = 40;

    // Safe distance to the back car in meter
    static constexpr double BACK_CLEARANCE_DIST = 15;
    
    // Current Lane ID
    int lane;

    // Lanes' informations
    vector<Lane> lane_infos;

    // Current velocity of the car
    double ref_vel;

    // Target velocity we want to achieve 
    // Always MAX_SPEED unless when we follow the car in front
    double target_vel;

    // Map waypoints
    vector<double> map_waypoints_x;
    vector<double> map_waypoints_y;
    vector<double> map_waypoints_s;
    vector<double> map_waypoints_dx;
    vector<double> map_waypoints_dy;

    // Current car state
    CarState car_state_;

    // Next state we want to drive
    NextState next_state_;

    // Update current car's state
    void UpdateCarState(json j);

    // Update All 3 Lane's information
    void UpdateLaneInfos(json sensor_fusion);

    // Calculate 5 anchor points
    vector<vector<double>> CalcAnchorPoints();

    // Chech if given Frenet d coordinate is in given lane
    bool IsInLane(double d_coord, int lane_id);

    // Get Lane ID(0/1/2) given Frenet d coordinate
    int GetLaneID(double d_cooord);

    // Convert Point in Car's local coordinate to Map coordinate
    vector<double> ConvertToMapCoordinate(double x, double y);

    // Convert Point in Map coordinate to Car's local coordinate
    vector<double> ConvertToCarCoordinate(double x, double y);
};

#endif