#include <fstream>
#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "json.hpp"
#include "spline.h"
#include "helpers.h"

using namespace std;

// for convenience
using json = nlohmann::json;

int main() {
  uWS::Hub h;

  // Load up map values for waypoint's x,y,s and d normalized normal vectors
  vector<double> map_waypoints_x;
  vector<double> map_waypoints_y;
  vector<double> map_waypoints_s;
  vector<double> map_waypoints_dx;
  vector<double> map_waypoints_dy;

  // read waypoint map from CSV
  string map_file_ = "../data/highway_map.csv";
  // The max s value before wrapping around the track back to 0
  double max_s = 6945.554;

  ifstream in_map_(map_file_.c_str(), ifstream::in);

  string line;
  while (getline(in_map_, line)) {
  	istringstream iss(line);
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

  //Start in lane 1 
  int lane = 1;

  //init reference velocity 
  double ref_vel = 0.0; // mph

  h.onMessage([&ref_vel, &lane, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,&map_waypoints_dx,&map_waypoints_dy]
    (uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length, uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    if (length && length > 2 && data[0] == '4' && data[1] == '2') {

      auto s = hasData(data);

      if (s != "") {
        auto j = json::parse(s);

        string event = j[0].get<string>();

        if (event == "telemetry") {
    
        	// Main car's localization Data
          	double car_x = j[1]["x"];
          	double car_y = j[1]["y"];
          	double car_s = j[1]["s"];
          	double car_d = j[1]["d"];
          	double car_yaw = j[1]["yaw"];
          	double car_speed = j[1]["speed"];

          	// Previous path data
          	auto previous_path_x = j[1]["previous_path_x"];
          	auto previous_path_y = j[1]["previous_path_y"];
          	// Previous path's end s and d values
          	double end_path_s = j[1]["end_path_s"];
          	double end_path_d = j[1]["end_path_d"];

          	// Sensor Fusion Data, a list of all other cars on the same side of the road.
          	auto sensor_fusion = j[1]["sensor_fusion"];

            // previous path point size.
        
            int prev_size = previous_path_x.size();

            if (prev_size > 0) {
              car_s = end_path_s;
            }

            // init states.
            bool car_ahead = false;
            bool car_left = false;
            bool car_righ = false;

            for ( int i = 0; i < sensor_fusion.size(); i++ ) 
            {   
                int car_lane= -1;
                //Obtain car[i] data from sensor fusion
                float d = sensor_fusion[i][6];
                // Find car[i]'s speed.
                double vx = sensor_fusion[i][3];
                double vy = sensor_fusion[i][4];
                //magnitude calculation
                double near_car_speed = sqrt(vx*vx + vy*vy);
                double near_car_s = sensor_fusion[i][5];
                
                // Predict car[i] position in 0.02 seconds later
                near_car_s += ((double)prev_size*0.02*near_car_speed);
                
                //Check if car[i] lane is in  front of ego car.
                if ( (d>4*lane) && (d<4*lane+4)) 
                {
                  car_ahead |= (near_car_s > car_s) && ((near_car_s - car_s) <= 30);  
                } 
                // check if car[i] in left lane
                else if ((d>4*lane-4) && (d<4*lane) )
                {
                    car_left |= ((car_s - near_car_s) <= 30)&&((near_car_s - car_s) <= 30);
                } 
              // check if car[i] in right lane
                else if ( (d>4*lane+4) && (d<4*lane+8)) 
                {
                    car_righ |= ((car_s - near_car_s) <= 30)&&((near_car_s - car_s) <= 30);
                }
            }
          
             // FSM algorithm for making driving decision
            if ( car_ahead ) 
            { 
              if ( !car_left && lane > 0 ) 
              {
                // if there is no car in left, change to left lance.
                
                lane = lane -1 ; // Change lane left.
              } 
              else if ( !car_righ && lane != 2 )
              {
                // if there is no car in right, change to right lance.
             
                lane = lane + 1; // Change lane to right.
              } 
              else 
              {
                // if car[i] ahead as well as  in left & reight lane, then reduce driving speed
                ref_vel-=0.224;
              }
            } 
            else 
             {
              if ( lane != 1 ) 
              { 
                if ( ( lane == 0 && !car_righ ) || ( lane == 2 && !car_left ) ) 
                { 
                  // if safe, back to center lane.
                  lane = 1;
                }
              }
              // vehicle speed controlling 
              if ( ref_vel >= 49.5 ) // max driving speed
              {
                ref_vel-=0.224;
                
              }
              else if ( ref_vel > 35 && ref_vel < 49.5)
               {
                ref_vel+=0.224;
               }
              else
              {
                ref_vel+=0.224*2;
              }
             }
        
            //Define vector template parameters for calculating
          	vector<double> ptsx;
            vector<double> ptsy;
            vector<double> next_x_vals;
            vector<double> next_y_vals;

            //Reference x,y yaw states
            double ref_x = car_x;
            double ref_y = car_y;
            double ref_yaw = deg2rad(car_yaw);

            //If previous size is almost empty , use the car as starting point
            if ( prev_size < 2 ) {
                /*use two points that make the path tangent to the car
                The velocity set as unit vector now*/
                double prev_car_x = car_x - cos(car_yaw);
                double prev_car_y = car_y - sin(car_yaw);

                //push back as ptsx in [prev_car_x1, car_x1, prev_car_x2, car_x2]
                ptsx.push_back(prev_car_x);
                ptsx.push_back(car_x);

                ptsy.push_back(prev_car_y);
                ptsy.push_back(car_y);
            } else {
                //Use the previous path's end points as starting reference
                ref_x = previous_path_x[prev_size - 1];
                ref_y = previous_path_y[prev_size - 1];

                double ref_x_prev = previous_path_x[prev_size - 2];
                double ref_y_prev = previous_path_y[prev_size - 2];

                //Use previous two points for calculating the reference yaw angle.
                ref_yaw = atan2(ref_y - ref_y_prev, ref_x - ref_x_prev);

                //Push the reference points to the ptsx,ptsy
                ptsx.push_back(ref_x_prev);
                ptsx.push_back(ref_x);

                ptsy.push_back(ref_y_prev);
                ptsy.push_back(ref_y);
            }

            /* In frenet add evenly 30 meters spaced points ahead of the staring reference
            The origin of the starting point is already calculated in above code. */
            vector<double> next_w0 = getXY(car_s + 30, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_w1 = getXY(car_s + 60, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);
            vector<double> next_w2 = getXY(car_s + 90, 2 + 4*lane, map_waypoints_s, map_waypoints_x, map_waypoints_y);

            // Add three far point for fitting spline. 
            ptsx.push_back(next_w0[0]);
            ptsx.push_back(next_w1[0]);
            ptsx.push_back(next_w2[0]);

            ptsy.push_back(next_w0[1]);
            ptsy.push_back(next_w1[1]);
            ptsy.push_back(next_w2[1]);

            // Making coordinates to local car coordinates.
            for ( int i = 0; i < ptsx.size(); i++ ) 
            {
              //shift car reference angle to 0 degree
              double shift_x = ptsx[i] - ref_x;
              double shift_y = ptsy[i] - ref_y;
          
              ptsx[i] = shift_x * cos(0 - ref_yaw) - shift_y * sin(0 - ref_yaw);
              ptsy[i] = shift_x * sin(0 - ref_yaw) + shift_y * cos(0 - ref_yaw);
            }

            // Create the spline.
            tk::spline s;
            // fitting the point into spline
            s.set_points(ptsx, ptsy);
 
            for ( int i = 0; i < prev_size; i++ ) {
              next_x_vals.push_back(previous_path_x[i]);
              next_y_vals.push_back(previous_path_y[i]);
            }

            /*Calculate how to break up spline points so that we travel at our desired reference velocity.
            */
            double target_x = 30.0;
            double target_y = s(target_x);
            double target_dist = sqrt(target_x*target_x + target_y*target_y);

            double x_add_on = 0;
             
            //generate 50 path points
            for( int i = 0; i <= 50 - prev_size; i++ ) 
            {
              
              //2.24 is distance between ref_point to target with the given velocity
              double N = target_dist/(0.02*ref_vel/2.24);
              double x_point = x_add_on + target_x/N;
              double y_point = s(x_point);

              x_add_on = x_point;

              double x_ref = x_point;
              double y_ref = y_point;
              
              //rotate and translate the point based on reference point.
             
              x_point = x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw);
              y_point = x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw);

              //Translation
              x_point += ref_x;
              y_point += ref_y;

              next_x_vals.push_back(x_point);
              next_y_vals.push_back(y_point);
            }

            json msgJson;

          	msgJson["next_x"] = next_x_vals;
          	msgJson["next_y"] = next_y_vals;

          	auto msg = "42[\"control\","+ msgJson.dump()+"]";

          
          	ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);

        }
      } else {
        
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });


  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
   
      res->end(nullptr, 0);
    }
  });

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
