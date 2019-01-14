#ifndef COST_H
#define COST_H
#include "vehicle.h"

using namespace std;

double calculate_cost(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, const vector<Vehicle> & trajectory);
double lane_change_cost(const Vehicle & vehicle,  const vector<Vehicle> & trajectory,  const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double inefficiency_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double danger_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);
double congestion_cost(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions, map<string, double> & data);

double lane_speed(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane);
double lane_danger(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane);
double lane_congestion(const Vehicle & vehicle, const map<int, vector<Vehicle>> & predictions, int lane);

map<string, double> get_helper_data(const Vehicle & vehicle, const vector<Vehicle> & trajectory, const map<int, vector<Vehicle>> & predictions);

#endif