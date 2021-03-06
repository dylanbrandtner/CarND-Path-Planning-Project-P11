#ifndef VEHICLE_H
#define VEHICLE_H
#include <iostream>
#include <random>
#include <vector>
#include <map>
#include <string>

using namespace std;

class Vehicle {
public:

  map<string, int> lane_direction = {{"PLCL", -1}, {"LCL", -1}, {"LCR", 1}, {"PLCR", 1}};

  double buffer; // impacts "keep lane" behavior.
  double danger_buffer = 10; // collision avoidance
  int lane_change_cooldown_orig;
  int lane_change_cooldown;

  int lane;

  double s;

  double v;

  double a;

  double target_speed;

  int lanes_available;

  double goal_s;

  string state;
  
  double horizon;

  /**
  * Constructor
  */
  Vehicle();
  Vehicle(int lane, double s, double v, double a, string state="CS", double horizon=2.0);

  /**
  * Destructor
  */
  virtual ~Vehicle();

  vector<Vehicle> choose_next_state(map<int, vector<Vehicle>> predictions);

  vector<string> successor_states();

  vector<Vehicle> generate_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<double> get_kinematics(map<int, vector<Vehicle>> predictions, int lane);

  vector<Vehicle> keep_lane_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  vector<Vehicle> prep_lane_change_trajectory(string state, map<int, vector<Vehicle>> predictions);

  double position_at(int t);

  bool get_vehicle_ahead(map<int, vector<Vehicle>> predictions, int lane, Vehicle & rVehicle);
  
  vector<Vehicle> generate_predictions();
};

#endif