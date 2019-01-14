#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const double LANE_CHANGE = pow(10, 1);
const double EFFICIENCY = pow(10, 5);
const double DANGER_COST = pow(10,7);


double lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases based on distance of intended lane 
    */
    double cost = 1 - 2*exp(-(abs(vehicle.lane - data["intended_lane"])));

    return cost;
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    */

    double proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
    double proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
    
    double cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;

    return cost;
}


double danger_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories vehicles in danger buffer
    */

    double current_lane_danger = lane_danger(vehicle, predictions, vehicle.lane);
    double proposed_lane_danger = lane_danger(vehicle, predictions, data["intended_lane"]);
    double final_lane_danger = lane_danger(vehicle, predictions, data["final_lane"]);
    
    double cost = 1 - 2*exp(-(abs(2.0*current_lane_danger - proposed_lane_danger - final_lane_danger)));

    return cost;
}

double lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Set lane speed to speed of nearest car ahead
    */
    double closest_vehicle_ahead = 10000;
    double lane_speed = 0;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        Vehicle veh = it->second[0];
        if (veh.lane == lane) {
            if (veh.s > vehicle.s && veh.s < closest_vehicle_ahead)
            {
                closest_vehicle_ahead = veh.s;
                lane_speed = veh.s;
            }
        }
    }
    
    if (lane_speed == 0)
    {
        return vehicle.target_speed;
    }
    
    return lane_speed;
}

double lane_danger(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Set lane speed to speed of nearest car ahead
    */
    double danger_val = 0;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        Vehicle veh = it->second[0];
        if (veh.lane == lane) {
            if (fabs(veh.s - vehicle.s) < vehicle.buffer)
            {
                danger_val++; // +1 for each car in buffer
                if (fabs(veh.s - vehicle.s) < vehicle.danger_buffer)
                {
                    danger_val += 15; // +15 for each car in danger buffer
                }
                
                if (veh.s < vehicle.s && veh.v > vehicle.v)
                {
                    danger_val += 3; // +3 for each car behind moving faster than ego car
                }                
            }
        }
    }
    
    return danger_val;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = {lane_change_cost, inefficiency_cost, };
    vector<double> weight_list = {LANE_CHANGE, EFFICIENCY, DANGER_COST};
    
    for (int i = 0; i < cf_list.size(); i++) {
        double new_cost = weight_list[i]*cf_list[i](vehicle, trajectory, predictions, trajectory_data);
        cost += new_cost;
    }

    return cost;

}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.
    distance_to_goal: the distance of the vehicle to the goal.

    Note that indended_lane and final_lane are both included to help differentiate between planning and executing
    a lane change in the cost functions.
    */
    map<string, double> trajectory_data;
    Vehicle trajectory_last = trajectory[1];
    double intended_lane;

    if (trajectory_last.state.compare("PLCL") == 0) {
        intended_lane = trajectory_last.lane - 1;
    } else if (trajectory_last.state.compare("PLCR") == 0) {
        intended_lane = trajectory_last.lane + 1;
    } else {
        intended_lane = trajectory_last.lane;
    }

    double distance_to_goal = vehicle.goal_s - trajectory_last.s;
    double final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    return trajectory_data;
}

