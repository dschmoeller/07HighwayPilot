#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;
using std::cout; 
using std::endl; 


int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // Waypoint map to read from
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  std::ifstream in_map_(map_file_.c_str(), std::ifstream::in);

  string line;
  while (getline(in_map_, line)) {
    std::istringstream iss(line);
    double x;
    double y;
    float s;
    float d_x;
    float d_y;
    iss >> x;
    iss >> y;
    iss >> s;
    iss >> d_x;
    iss >> d_y;
    map_waypoints_x.push_back(x);
    map_waypoints_y.push_back(y);
    map_waypoints_s.push_back(s);
    map_waypoints_dx.push_back(d_x);
    map_waypoints_dy.push_back(d_y);
  }

  h.onMessage([&map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
               &map_waypoints_dx,&map_waypoints_dy]
              (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
               uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);
        
        string event = j[0].get<string>();
        
        if (event == "telemetry") {
          // j[1] is the data JSON object
          
          // Main car's localization Data
          double car_x = j[1]["x"];
          double car_y = j[1]["y"];
          double car_s = j[1]["s"];
          double car_d = j[1]["d"];
          double car_yaw = j[1]["yaw"];
          double car_speed = j[1]["speed"];

          // Previous path data given to the Planner
          auto previous_path_x = j[1]["previous_path_x"];
          auto previous_path_y = j[1]["previous_path_y"];
          // Previous path's end s and d values 
          double end_path_s = j[1]["end_path_s"];
          double end_path_d = j[1]["end_path_d"];

          // Sensor Fusion Data, a list of all other cars on the same side 
          //   of the road.
          auto sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          
          
          // ###########################################################################
          /*
          double pos_s;
          double pos_d;
          double pos_x; 
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }
          if (path_size == 0) {
            pos_s = car_s;
            pos_d = car_d;  
          } else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);

            vector<double> pos_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y); 
            pos_s = pos_sd[0]; 
            pos_d = pos_sd[1]; 
          }

          double sample_time = 0.02; 
          double v_max = 20;
          double a_max = 9;
          double delta_v_max = a_max*sample_time; // 0.02 [m/s]
          double dist_inc = 0;
          double v_next_max = 0;     
          for (int i = path_size; i < 100; ++i) {
            // Adapt dist_inc for each iteration
            double vel = (i+1)*delta_v_max; 
            if (vel > v_max){ vel = v_max; }
            dist_inc = vel*sample_time; 
            // Increment pos_s in each iteration
            pos_s += dist_inc;

            vector<double> pos_xy = getXY(pos_s, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double x_target = pos_xy[0]; 
            double y_target = pos_xy[1];   
            next_x_vals.push_back(x_target);
            next_y_vals.push_back(y_target);
          }*/
          // ###########################################################################
          
          
          
          
          
          
          
          
          // Variable Definitions
          int number_pts = 100; 
          double pos_s;
          double pos_s_dot; 
          double pos_s_dot_dot; 
          double pos_d;
          double pos_x; 
          double pos_y;
          double angle;
          int path_size = previous_path_x.size();
          vector<double> s_dots (number_pts, 0); 
          vector<double> s_dots_dots (number_pts, 0); 

          // Write already calculated points back
          for (int i = 0; i < path_size; ++i) {
            next_x_vals.push_back(previous_path_x[i]);
            next_y_vals.push_back(previous_path_y[i]);
          }

          vector<double> coeff_s(6, 0);   
          // Distinguish start from running mode
          if (path_size == 0) {
            pos_s = car_s;
            pos_s_dot = 0; 
            pos_s_dot_dot = 0; 
            pos_d = car_d;
            coeff_s[1] = 0; 
            coeff_s[2] = 1; 
            coeff_s[3] = 0; 
            coeff_s[4] = 0; 
            coeff_s[5] = 0;   
          } 
          else {
            pos_x = previous_path_x[path_size-1];
            pos_y = previous_path_y[path_size-1];
            double pos_x2 = previous_path_x[path_size-2];
            double pos_y2 = previous_path_y[path_size-2];
            angle = atan2(pos_y-pos_y2,pos_x-pos_x2);
            vector<double> pos_sd = getFrenet(pos_x, pos_y, angle, map_waypoints_x, map_waypoints_y); 
            pos_s = pos_sd[0]; 
            pos_d = pos_sd[1]; 
            pos_s_dot = s_dots[path_size - 1]; 
            pos_s_dot_dot = s_dots_dots[path_size - 1];
            coeff_s[1] = 1; 
            coeff_s[2] = 0; 
            coeff_s[3] = 0; 
            coeff_s[4] = 0; 
            coeff_s[5] = 0;  
          }

          // Define conditions for trajectory generation
          double v_max = 20; // [m/s]
          double sample_time = 0.1;
          double duration = 10;  
          vector<double> s_i = {pos_s, pos_s_dot, pos_s_dot_dot}; 
          vector<double> s_f = {pos_s + 50, v_max, 0};  
          //vector <double> coeff_s = JMT(s_i, s_f, duration); 
          coeff_s[0] = pos_s; 
          cout << endl << "Use the following trajectory coefficients" << endl; 
          for (auto c : coeff_s){
            cout << c << " "; 
          }
          cout << endl;   

          // Use min jerk trajectory coefficients to sample points     
          cout << endl << "Pushing " << number_pts - path_size << "Elements to the list" << endl; 
          for (int i = 0; i < (number_pts - path_size); ++i) {
            // Sample s points
            double s = point_gen(coeff_s, (i+1)*sample_time); 
            cout << s << " "; 
            // Sample velocity and update corresponding vector
            double s_dot = vel_gen(coeff_s, (i+1)*sample_time);
            s_dots.push_back(s_dot);
            s_dots.erase(s_dots.begin());     
            // Sample acceleration and update corresponding vector
            double s_dot_dot = acc_gen(coeff_s, (i+1)*sample_time); 
            s_dots_dots.push_back(s_dot_dot);
            s_dots_dots.erase(s_dots_dots.begin()); 

            // Transform back to x and y space 
            // Push values to next_vals vector
            vector<double> pos_xy = getXY(s, pos_d, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            double x_target = pos_xy[0]; 
            double y_target = pos_xy[1];   
            next_x_vals.push_back(x_target);
            next_y_vals.push_back(y_target);
          }

         

        
          




          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"control\","+ msgJson.dump()+"]";

          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  }); // end h.onMessage

  h.onConnection([&h](uWS::WebSocket<uWS::SERVER> ws, uWS::HttpRequest req) {
    std::cout << "Connected!!!" << std::endl;
  });

  h.onDisconnection([&h](uWS::WebSocket<uWS::SERVER> ws, int code,
                         char *message, size_t length) {
    ws.close();
    std::cout << "Disconnected" << std::endl;
  });

  int port = 4567;
  if (h.listen(port)) {
    std::cout << "Listening to port " << port << std::endl;
  } else {
    std::cerr << "Failed to listen to port" << std::endl;
    return -1;
  }
  
  h.run();
}