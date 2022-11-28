#include "path_planner.h"
#include "spline.h"

PathPlanner::PathPlanner() {
    lane = CENTER_LANE;
    lane_infos.resize(N_LANES);
    ref_vel = 0.0; 
    next_state_ = KEEP_LANE;
}

bool PathPlanner::IsInLane(double d_coord, int lane_id) {
    // Check if Frenet d coordinate falls into given lane
    return (d_coord > (LANE_WIDTH * lane_id) && d_coord < (LANE_WIDTH * (lane_id + 1)));
}

int PathPlanner::GetLaneID(double d_coord){
    // Find which lane car is in using Frenet d coordinate
    if(d_coord >=0 && d_coord < LANE_WIDTH){
        return 0;
    }
    if(d_coord >=LANE_WIDTH && d_coord < LANE_WIDTH*2){
        return 1;
    }
    if(d_coord >=LANE_WIDTH*2 && d_coord <= LANE_WIDTH*3){
        return 2;
    }
    return -1;
}

void PathPlanner::UpdateLaneInfos(json sensor_fusion) {
    // Reset previous lane infos beforehand
    for(int i=0; i<N_LANES; i++) {
        lane_infos[i].is_free = true;
        lane_infos[i].car_in_front = false;
        lane_infos[i].car_in_back = false;
        lane_infos[i].front_car_distance = 5000;
        lane_infos[i].back_car_distance = -5000;
    }

    for(int i=0; i<sensor_fusion.size(); i++) { 
        // Parse each detection result in the sensor fusion
        float d = sensor_fusion[i][6]; 
        double vx = sensor_fusion[i][3]; 
        double vy = sensor_fusion[i][4]; 
        double check_speed = sqrt(vx*vx + vy*vy); 
        double check_car_s = sensor_fusion[i][5]; 
        double check_distance = check_car_s - car_state_.s; 
        
        int lane_id = GetLaneID(d);
        if(lane_id != -1){
            if(check_distance > 0) { // If the detected car is in the front
                lane_infos[lane_id].car_in_front = true;
                if(check_distance < lane_infos[lane_id].front_car_distance){
                    lane_infos[lane_id].front_car_distance = check_distance;
                    lane_infos[lane_id].front_car_speed = check_speed;
                    if(check_distance < FRONT_CLEARANCE_DIST)  {
                        lane_infos[lane_id].is_free = false;
                    }
                } 
            } else { // If the detected car is in the back
                lane_infos[lane_id].car_in_back = true;
                if(check_distance > lane_infos[lane_id].back_car_distance){
                    lane_infos[lane_id].back_car_distance = check_distance;
                    lane_infos[lane_id].back_car_speed = check_speed;
                    if(check_distance > -BACK_CLEARANCE_DIST)  {
                        lane_infos[lane_id].is_free = false;
                    }
                } 
            }
        }
   
    }
    /*for(auto& l : lane_infos) {
        std::cout << l.car_in_front << " " << l.car_in_back << " " << l.is_free << " || ";
    }
    std::cout << std::endl;*/
}

void PathPlanner::UpdateCarState(json j){
    // Ego car's localization Data
    car_state_.x = j["x"];
    car_state_.y = j["y"];
    car_state_.s = j["s"];
    car_state_.d = j["d"];
    car_state_.yaw = j["yaw"];
    car_state_.speed = j["speed"];
    
    // Reference pose for planning next path
    car_state_.ref_x = car_state_.x;
    car_state_.ref_y = car_state_.y;
    car_state_.ref_s = car_state_.s;
    car_state_.ref_yaw = deg2rad(car_state_.yaw);

    // Previous path data given to the Planner
    car_state_.previous_path_x = j["previous_path_x"];
    car_state_.previous_path_y = j["previous_path_y"];
    
    // Previous path's end s and d values 
    car_state_.end_path_s = j["end_path_s"];
    car_state_.end_path_d = j["end_path_d"];
}

vector<vector<double>> PathPlanner::CalcAnchorPoints() {
    // Create List of 5 points for later used with spline
    // To smoothly continue current path 2 points from previous path and 3 future points are collected
    vector<double> ptsx;
    vector<double> ptsy; 
    
    double prev_ref_x;
    double prev_ref_y;

    int prev_size = car_state_.previous_path_x.size();
    
    // First add 2 anchor points from the previous path's ending
    if(prev_size < 2) { 
        // If previous size is almost empty, use current car pose as reference
        car_state_.ref_x = car_state_.x;
        car_state_.ref_y = car_state_.y;
        car_state_.ref_s = car_state_.s; 
        car_state_.ref_yaw = deg2rad(car_state_.yaw);

        prev_ref_x = car_state_.ref_x - cos(car_state_.ref_yaw);
        prev_ref_y = car_state_.ref_y - sin(car_state_.ref_yaw); 

    } else { 
        // Use the previous path's end point as starting reference
        car_state_.ref_x = car_state_.previous_path_x[prev_size-1];
        car_state_.ref_y = car_state_.previous_path_y[prev_size-1]; 
        car_state_.ref_s = car_state_.end_path_s; 

        prev_ref_x = car_state_.previous_path_x[prev_size-2];
        prev_ref_y = car_state_.previous_path_y[prev_size-2];
        car_state_.ref_yaw = atan2(car_state_.ref_y - prev_ref_y, car_state_.ref_x - prev_ref_x); 

    } 

    // Add 3 more anchor points 30m, 60m, 90m down the road from reference position
    vector<double> next_wp0 = getXY(car_state_.ref_s + 30, (LANE_WIDTH*lane + LANE_WIDTH/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp1 = getXY(car_state_.ref_s + 60, (LANE_WIDTH*lane + LANE_WIDTH/2), map_waypoints_s, map_waypoints_x, map_waypoints_y);
    vector<double> next_wp2 = getXY(car_state_.ref_s + 90, (LANE_WIDTH*lane + LANE_WIDTH/2), map_waypoints_s, map_waypoints_x, map_waypoints_y); 

    // push x points
    ptsx.push_back(prev_ref_x);
    ptsx.push_back(car_state_.ref_x); 
    ptsx.push_back(next_wp0[0]);
    ptsx.push_back(next_wp1[0]);
    ptsx.push_back(next_wp2[0]); 
    // push y points
    ptsy.push_back(prev_ref_y);
    ptsy.push_back(car_state_.ref_y); 
    ptsy.push_back(next_wp0[1]);
    ptsy.push_back(next_wp1[1]);
    ptsy.push_back(next_wp2[1]);

    // Transform coordinates to ego car's local coordinate system
    for(int i=0; i<ptsx.size(); i++) 
    { 
        double shift_x = ptsx[i] - car_state_.ref_x;
        double shift_y = ptsy[i] - car_state_.ref_y; 

        ptsx[i] = (shift_x * cos(0-car_state_.ref_yaw) - shift_y * sin(0-car_state_.ref_yaw));
        ptsy[i] = (shift_x * sin(0-car_state_.ref_yaw) + shift_y * cos(0-car_state_.ref_yaw));
    }

    return {ptsx, ptsy};
}

void PathPlanner::UpdatePath(json j) {
    // Update current car state and lanes' traffic info
    UpdateCarState(j);
    UpdateLaneInfos(j["sensor_fusion"]);

    next_x_vals.clear();
    next_y_vals.clear();

    double target_vel;
    bool too_close = lane_infos[lane].car_in_front && lane_infos[lane].front_car_distance < FRONT_CLEARANCE_DIST;

    bool left_lane_free = false;
    if(lane != LEFT_LANE) { // If the car is not currently in left lane
        left_lane_free = lane_infos[lane-1].is_free;
    }

    bool right_lane_free = false;
    if(lane != RIGHT_LANE) { // If the car is not currently in right lane
        right_lane_free = lane_infos[lane+1].is_free;
    }

    //std::cout << lane << ": " << too_close << " " << left_lane_free << " " << right_lane_free << std::endl;
 
    // State transition logic   
    if(too_close) {
        if(right_lane_free && left_lane_free) {
            if(lane_infos[RIGHT_LANE].front_car_distance >= lane_infos[LEFT_LANE].front_car_distance) {
                next_state_ = CHANGE_LANE_RIGHT;
            } else {
                next_state_ = CHANGE_LANE_LEFT;
            }
        } else if(right_lane_free){
            next_state_ = CHANGE_LANE_RIGHT;
        } else if(left_lane_free) {
            next_state_ = CHANGE_LANE_LEFT;
        } else {
            next_state_ = PREPARE_CHANGE_LANE;
        }
    } else {
        next_state_ = KEEP_LANE;
    }

    switch (next_state_) {
        case KEEP_LANE:
            target_vel = MAX_SPEED;
            std::cout << "Keep going at Lane #" << lane << std::endl;
            break;
        case PREPARE_CHANGE_LANE:
            // Keep same speed with the front car
            target_vel = target_vel = lane_infos[lane].front_car_speed; 
            std::cout << "Looking to change lane... " << std::endl;
            break;
        case CHANGE_LANE_LEFT:
            lane--;
            target_vel = MAX_SPEED;
            std::cout << "Changing to left Lane #" << lane << std::endl;
            break;
        case CHANGE_LANE_RIGHT:
            lane++;
            target_vel = MAX_SPEED;
            std::cout << "Changing to right Lane #" << lane << std::endl;
            break;
        default:
            break;
    }

    // Create Sparse Points for creating spline (Points are in car coordinate)
    vector<vector<double>> anchor_pts = CalcAnchorPoints();

    // Create a spline using 5 anchor points
    tk::spline s;
    s.set_points(anchor_pts[0], anchor_pts[1]);

    int prev_size = car_state_.previous_path_x.size();
    int n_remaining_pts = N_PATH_POINTS - prev_size;
    
    // Add remaining points from previous path to new path
    for (int i=0; i<prev_size; i++) {
        next_x_vals.push_back(car_state_.previous_path_x[i]);
        next_y_vals.push_back(car_state_.previous_path_y[i]);
    } 

    // Fill the remainder of the new path by interpolating using spline
    double target_x = TIME_STEP * N_PATH_POINTS * MAX_SPEED;
    double target_y = s(target_x);
    double target_dist = sqrt((target_x)*(target_x)+(target_y)*(target_y)); 
    // Ratio/projedction of x_shift vs total distance 
    double target_x_ratio = target_x / target_dist;    
    double prev_x = 0.0;

    for(int i=0; i<n_remaining_pts; i++) { 
        if(ref_vel > target_vel || lane_infos[lane].front_car_distance < FRONT_CLEARANCE_DIST/2) {
            ref_vel -= MAX_VEL_CHANGE_PER_STEP;
        } else if(ref_vel < target_vel) {
            ref_vel += MAX_VEL_CHANGE_PER_STEP;
        }

        double x_point = prev_x + TIME_STEP * ref_vel * target_x_ratio ; 
        double y_point = s(x_point); 
        prev_x = x_point;
        
        auto map_pts = ConvertToMapCoordinate(x_point, y_point);

        next_x_vals.push_back(map_pts[0]);
        next_y_vals.push_back(map_pts[1]); 
    }
}

vector<double> PathPlanner::ConvertToMapCoordinate(double x, double y) {
    // Convert from car's local coordinate to map coordinate
    double x_map = x * cos(car_state_.ref_yaw) - y * sin(car_state_.ref_yaw) + car_state_.ref_x; 
    double y_map = x * sin(car_state_.ref_yaw) + y * cos(car_state_.ref_yaw) + car_state_.ref_y; 
    return {x_map, y_map};
}

vector<double> PathPlanner::ConvertToCarCoordinate(double x, double y) {
    // Convert from map coordinate to car's local coordinate
    double shift_x = x - car_state_.ref_x;
    double shift_y = y - car_state_.ref_y; 

    double car_x = (shift_x *cos(0-car_state_.ref_yaw) - shift_y*sin(0-car_state_.ref_yaw));
    double car_y = (shift_x *sin(0-car_state_.ref_yaw) + shift_y*cos(0-car_state_.ref_yaw));
    return {car_x, car_y};
}

void PathPlanner::SetMap(vector<double>& x, vector<double>& y, vector<double>& s, vector<double>& dx, vector<double>& dy) {
    map_waypoints_x = x;
    map_waypoints_y = y;
    map_waypoints_s = s;
    map_waypoints_dx = dx;
    map_waypoints_dy = dy;
}