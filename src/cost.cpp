#include "cost.h"
#include "vehicle.h"
#include <functional>
#include <iterator>
#include <map>
#include <math.h>

const double LANE_CHANGE = pow(10, 4);
const double EFFICIENCY = pow(10, 5);
const double CONGESTION_COST = pow(10,5);
const double DANGER_COST = pow(10,8);

double lane_change_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost increases for lane changes
    */
    
    if (vehicle.lane == data["intended_lane"])
    {
        return 0;
    }
    else
    {
        return 1;
    }
}

double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with intended lane and final lane that have traffic slower than vehicle's target speed. 
    */

    double proposed_speed_intended = lane_speed(vehicle, predictions, data["intended_lane"]);
    double proposed_speed_final = lane_speed(vehicle, predictions, data["final_lane"]);
    
    double cost = (2.0*vehicle.target_speed - proposed_speed_intended - proposed_speed_final)/vehicle.target_speed;
    //std::cout << "inefficiency_cost: " << vehicle.target_speed << "," << proposed_speed_intended << "," << proposed_speed_final << "," << std::endl;

    return cost;
}


double danger_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with vehicles in danger buffer
    */

    double proposed_lane_danger = lane_danger(vehicle, predictions, data["intended_lane"]);
    
    double cost = 1 - exp(-(abs(proposed_lane_danger)));

    return cost;
}

double congestion_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data) {
    /*
    Cost becomes higher for trajectories with vehicles in buffer
    */

    double proposed_lane_congestion = lane_congestion(vehicle, predictions, data["intended_lane"]);
    
    double cost = 1 - exp(-(abs(proposed_lane_congestion)));

    return cost;
}

double lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Set lane speed to speed of nearest car ahead within twice the buffer
    */
    double closest_vehicle_ahead = 10000;
    double lane_speed = 0;
    //std::cout << "vehicle.s: " << vehicle.s << std::endl;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        Vehicle veh = it->second[0];
        //std::cout << "veh.lane: " << veh.lane << " veh.s: " << veh.s << " veh.v: " << veh.v << std::endl;
        if (veh.lane == lane) // check for given lane
        {
            // for cars ahead, within twice the buffer, find the nearest one
            if (veh.s > vehicle.s && veh.s < closest_vehicle_ahead && ((veh.s - vehicle.s) < vehicle.buffer*2) )
            {                
                closest_vehicle_ahead = veh.s;
                lane_speed = veh.v;
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
    Amount of vehicles in danger buffer
    */
    double danger_val = 0;
    double ego_s = 0;
    //std::cout << "vehicle.s: " << vehicle.s << std::endl;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        for (int i=0;i < it->second.size();i++)
        {
            Vehicle veh = it->second[i];
            ego_s = vehicle.s + vehicle.v*i + vehicle.a*i*i/2.0;
            //std::cout << "veh.lane: " << veh.lane << " veh.s: " << veh.s << " veh.v: " << veh.v << std::endl;
            if (veh.lane == lane) {
                if (fabs(veh.s - ego_s) < vehicle.danger_buffer) 
                {
                    //std::cout << "vehicle found in danger buffer for lane " << lane << ", vehicle.s: " << vehicle.s << " veh.s: " << veh.s << std::endl;
                    danger_val+= (vehicle.danger_buffer - fabs(veh.s - ego_s)); // add based on distance
                }
            }
        }
    }
    
    return danger_val;
}

double lane_congestion(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane) {
    /*
    Amount of vehicles in buffer + vehicles behind with higher speeds
    */
    double cong_val = 0;
    for (map<int, vector<Vehicle>>::const_iterator it = predictions.begin(); it != predictions.end(); ++it) {
        Vehicle veh = it->second[0];
        if (veh.lane == lane) {
            if (fabs(veh.s - vehicle.s) < vehicle.buffer)
            {
                cong_val++; // +1 for each car in buffer
                if (veh.s < vehicle.s && veh.v > vehicle.v)
                {
                    cong_val++; // +2 for each car behind moving faster than ego car
                }
            }
        }
    }
    
    return cong_val;
}

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory) {
    /*
    Sum weighted cost functions to get total cost for trajectory.
    */
    map<string, double> trajectory_data = get_helper_data(vehicle, trajectory, predictions);
    double cost = 0.0;

    //Add additional cost functions here.
    vector< function<double(const Vehicle & , const vector<Vehicle> &, const map<int, vector<Vehicle>> &, map<string, double> &)>> cf_list = {lane_change_cost, inefficiency_cost, danger_cost, congestion_cost};
    vector<double> weight_list = {LANE_CHANGE, EFFICIENCY, DANGER_COST, CONGESTION_COST};
    
    for (int i = 0; i < cf_list.size(); i++) {
        double cost_item = cf_list[i](vehicle, trajectory, predictions, trajectory_data);
       // std::cout << "cost: " << cost_item << std::endl;
        double new_cost = weight_list[i]*cost_item;
        cost += new_cost;
    }
    //std::cout << "---------------"  << std::endl;

    return cost;

}

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions) {
    /*
    Generate helper data to use in cost functions:
    indended_lane: the current lane +/- 1 if vehicle is planning or executing a lane change.
    final_lane: the lane of the vehicle at the end of the trajectory.

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

    double final_lane = trajectory_last.lane;
    trajectory_data["intended_lane"] = intended_lane;
    trajectory_data["final_lane"] = final_lane;
    return trajectory_data;
}

