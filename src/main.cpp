#include <uWS/uWS.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "helpers.h"
#include "json.hpp"
#include "spline.h"


#define MAX_SPEED 49.5
#define LANE_CHANGE_SPEED 35
#define MAX_ALLOWED_S_FOR_LANE_CHANGE 10
#define MIN_ALLOWED_S_FOR_NEIGHBOUR_CAR 50
enum system_state
{
	DRIVE_FORWARD,
	TRACK_LEADING_CAR,
	CHANGE_LANE_LEFT,
	CHANGE_LANE_RIGHT
};

static char* enumStrings[] = { "DRIVE_FORWARD", "TRACK_LEADING_CAR",
                               "CHANGE_LANE_LEFT", "CHANGE_LANE_RIGHT"};

enum change_lane_direction
{
	CHANGE_LANE_TO_LEFT,
	CHANGE_LANE_TO_RIGHT
};
// for convenience
using nlohmann::json;
using std::string;
using std::vector;

system_state state = DRIVE_FORWARD;
int leading_car_id;
bool is_leading_car_dissappeared = false;
int lane_id = 1;
bool is_lane_changed = false;

vector<double> get_car_by_id(int id, vector<vector<double>> &sensor_fusion)
{
	vector<double> ret_car;
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		if(sensor_fusion[i][0] == id)
		{
			ret_car = sensor_fusion[i];
		}
	}

	return ret_car;
}

void run_drive_forward(double &ref_vel)
{
	ref_vel = MAX_SPEED;
}

bool leading_car_detected(vector<vector<double>> &sensor_fusion,
		int lane_id, int prev_size, double car_s)
{
	bool ret_val = false;
	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		if (d > 4*lane_id && d < 4*lane_id + 4)
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt((vx * vx) + (vy * vy));
			double check_car_s = sensor_fusion[i][5];

			check_car_s += (double) prev_size * .02 * check_speed;

			if(check_car_s > car_s && check_car_s - car_s < 50)
			{

				leading_car_id = sensor_fusion[i][0];
				ret_val = true;

			}
		}
	}
	return ret_val;

}

void run_track_leading_car(vector<vector<double>> &sensor_fusion , int prev_size, double car_s,
		double &target_vel ,int lane_id)
{

	bool is_car_detected = false;

	for (int i = 0; i < sensor_fusion.size(); i++)
	{
		float d = sensor_fusion[i][6];
		if (d > 4*lane_id && d < 4*lane_id + 4)
		{
			double vx = sensor_fusion[i][3];
			double vy = sensor_fusion[i][4];
			double check_speed = sqrt((vx * vx) + (vy * vy));
			double check_car_s = sensor_fusion[i][5];

			check_car_s += (double) prev_size * .02 * check_speed;

			if(check_car_s > car_s && check_car_s - car_s < 50)
			{

				leading_car_id = sensor_fusion[i][0];
				is_car_detected = true;
				target_vel = check_speed * 2.237;
			}
		}
	}

	if(!is_car_detected)
	{
		is_leading_car_dissappeared = true;
	}

}

bool change_lane_available(change_lane_direction direction, vector<vector<double>> &sensor_fusion , int lane_id, int prev_size, double car_s)
{
	bool ret_val = false;
	if((direction == CHANGE_LANE_TO_LEFT &&lane_id > 0) || (direction == CHANGE_LANE_TO_RIGHT && lane_id < 2))
	{
		int target_lane_id;
		vector<vector<double>> cars_in_target_lane;


		if(direction == CHANGE_LANE_TO_LEFT)
		{
			target_lane_id = lane_id - 1;
		}
		else if(direction == CHANGE_LANE_TO_RIGHT)
		{
			target_lane_id = lane_id + 1;
		}

		for (int i = 0; i < sensor_fusion.size(); i++)
		{
			float d = sensor_fusion[i][6];
			if (d > 4*target_lane_id && d < 4*target_lane_id + 4)
			{
				cars_in_target_lane.push_back(sensor_fusion[i]);
			}
		}

		ret_val = true;
		double min_s = 100000;
		for (int i = 0; i < cars_in_target_lane.size(); i++)
		{
			double vx = cars_in_target_lane[i][3];
			double vy = cars_in_target_lane[i][4];
			double check_speed = sqrt((vx * vx) + (vy * vy));
			double check_car_s = cars_in_target_lane[i][5];

			check_car_s += (double) prev_size * .02 * check_speed;

			if(check_car_s < min_s)
			{
				min_s = check_car_s;
			}
			if(check_car_s <= car_s + MIN_ALLOWED_S_FOR_NEIGHBOUR_CAR &&
					check_car_s >= car_s - MAX_ALLOWED_S_FOR_LANE_CHANGE)
			{
				ret_val = false;
			}

		}
//		vector<double> leading_car = get_car_by_id(leading_car_id, sensor_fusion);
//		ret_val = ret_val &&
//				(leading_car[2] - car_s > MAX_ALLOWED_S_FOR_LANE_CHANGE);

	}

	return ret_val;
}

bool lane_changed(int target_lane_id, double car_d)
{
	bool ret_val = false;
	if (car_d > 4*target_lane_id && car_d < 4*target_lane_id + 4)
	{
		ret_val = true;
	}
	return ret_val;
}

void run_statemachine(vector<vector<double>> &sensor_fusion,
		int &lane_id, int prev_size, double car_s,double car_d, double &target_vel)
{
	switch(state)
	{
	case DRIVE_FORWARD:
		run_drive_forward(target_vel);
		if (leading_car_detected(sensor_fusion,lane_id,prev_size,car_s))
		{
			state = TRACK_LEADING_CAR;
			printf("state = %s \n", enumStrings[state]);
			printf("lane_id = %d \n", lane_id);
		}

		break;

	case TRACK_LEADING_CAR:
		run_track_leading_car(sensor_fusion , prev_size, car_s, target_vel ,lane_id);
		if(is_leading_car_dissappeared)
		{
			state = DRIVE_FORWARD;
			is_leading_car_dissappeared = false;
			printf("state = %s \n", enumStrings[state]);
			printf("lane_id = %d \n", lane_id);
		}
		else if(change_lane_available(CHANGE_LANE_TO_LEFT,sensor_fusion ,lane_id,prev_size,car_s))
		{
			state = CHANGE_LANE_LEFT;
			printf("state = %s \n", enumStrings[state]);
			printf("lane_id = %d \n", lane_id);
		}
		else if (change_lane_available(CHANGE_LANE_TO_RIGHT,sensor_fusion ,lane_id,prev_size,car_s))
		{
			state = CHANGE_LANE_RIGHT;
			printf("state = %s \n", enumStrings[state]);
			printf("lane_id = %d \n", lane_id);
		}
		break;

	case CHANGE_LANE_LEFT:
		if (!is_lane_changed)
		{
			lane_id = lane_id - 1;
			is_lane_changed = true;
		}
		if(target_vel < LANE_CHANGE_SPEED)
		{
			target_vel = LANE_CHANGE_SPEED;
		}

		if (lane_changed(lane_id, car_d))
		{
			state = DRIVE_FORWARD;
			is_lane_changed = false;
			printf("state = %s \n", enumStrings[state]);
			printf("lane_id = %d \n", lane_id);
		}

		break;

	case CHANGE_LANE_RIGHT:

		if (!is_lane_changed)
		{
			lane_id = lane_id + 1;
			is_lane_changed = true;
		}
		if(target_vel < LANE_CHANGE_SPEED)
		{
			target_vel = LANE_CHANGE_SPEED;
		}
		if (lane_changed(lane_id, car_d))
		{
			state = DRIVE_FORWARD;
			is_lane_changed = false;
			printf("state = %s \n", enumStrings[state]);
			printf("lane_id = %d \n", lane_id);
		}

		break;
	}

}


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
  double ref_vel = 0;
  double target_vel = MAX_SPEED;

  h.onMessage([&target_vel ,&ref_vel, &map_waypoints_x,&map_waypoints_y,&map_waypoints_s,
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
          vector<vector<double>> sensor_fusion = j[1]["sensor_fusion"];

          json msgJson;



          int prev_size = previous_path_x.size();

          run_statemachine(sensor_fusion,lane_id, prev_size, car_s, car_d, target_vel);


          vector<double> ptsx;
          vector<double> ptsy;

          vector<double> next_x_vals;
          vector<double> next_y_vals;

          /**
           * TODO: define a path made up of (x,y) points that the car will visit
           *   sequentially every .02 seconds
           */


          double ref_x = car_x;
          double ref_y = car_y;
          double ref_yaw = deg2rad(car_yaw);

          if (prev_size < 2)
          {
        	  double prev_car_x = car_x - cos(car_yaw);
        	  double prev_car_y = car_y - sin(car_yaw);

        	  ptsx.push_back(prev_car_x);
        	  ptsx.push_back(car_x);

        	  ptsy.push_back(prev_car_y);
			  ptsy.push_back(car_y);
          }
          else
          {
        	  ref_x = previous_path_x[prev_size - 1];
        	  ref_y = previous_path_y[prev_size - 1];

        	  double prev_ref_x = previous_path_x[prev_size - 2];
        	  double prev_ref_y = previous_path_y[prev_size - 2];

        	  ref_yaw = atan2(ref_y - prev_ref_y , ref_x - prev_ref_x );

        	  ptsx.push_back(prev_ref_x);
        	  ptsx.push_back(ref_x);

        	  ptsy.push_back(prev_ref_y);
        	  ptsy.push_back(ref_y);
          }


          vector<double> next_wp0 = getXY(car_s + 50, (2+4*lane_id), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp1 = getXY(car_s + 100, (2+4*lane_id), map_waypoints_s, map_waypoints_x, map_waypoints_y);
          vector<double> next_wp2 = getXY(car_s + 150, (2+4*lane_id), map_waypoints_s, map_waypoints_x, map_waypoints_y);

          ptsx.push_back(next_wp0[0]);
          ptsx.push_back(next_wp1[0]);
          ptsx.push_back(next_wp2[0]);


          ptsy.push_back(next_wp0[1]);
          ptsy.push_back(next_wp1[1]);
          ptsy.push_back(next_wp2[1]);

          //transform the pts to the vehicle coordinates
          for (int i = 0 ; i < ptsx.size(); i++)
          {
        	  double shift_x = ptsx[i] - ref_x;
        	  double shift_y = ptsy[i] - ref_y;

        	  ptsx[i] = (shift_x *cos(0 - ref_yaw) - shift_y * sin(0-ref_yaw));
        	  ptsy[i] = (shift_x *sin(0 - ref_yaw) + shift_y * cos(0-ref_yaw));

          }

          tk::spline s;

          s.set_points(ptsx, ptsy);

          //fill the path with the previous not made waypoints

          for (int i = 0; i < previous_path_x.size(); i ++)
          {
        	  next_x_vals.push_back(previous_path_x[i]);
        	  next_y_vals.push_back(previous_path_y[i]);
          }

          double target_x = 30;
          double target_y = s(target_x);
          double target_dist = sqrt((target_x) * (target_x) + (target_y) * (target_y));
          double x_addon = 0;


          for (int i = 1; i <= 50-previous_path_x.size(); i++)
          {
//        	  if (too_close)
//        	  {
//        		  ref_vel -= .1;
//        		  if(ref_vel <= 0)
//        		  {
//        			  ref_vel = 1;
//        		  }
//        	  }
        	  if(ref_vel > target_vel)
        	  {
        		  ref_vel -= .224;
        	  }
        	  else if (ref_vel < target_vel)
        	  {
        		  ref_vel += .224;
        	  }
//        	  printf("ref_vel = %f \n", ref_vel);
        	  double N = (target_dist/(.02*ref_vel/2.24));
        	  double x_point = x_addon + (target_x/N);
        	  double y_point = s(x_point);

        	  x_addon = x_point;

        	  double x_ref = x_point;
        	  double y_ref = y_point;

        	  //get back to global coordinates
        	  x_point = (x_ref * cos(ref_yaw) - y_ref * sin(ref_yaw));
        	  y_point = (x_ref * sin(ref_yaw) + y_ref * cos(ref_yaw));

        	  x_point+= ref_x;
        	  y_point+= ref_y;

        	  next_x_vals.push_back(x_point);
        	  next_y_vals.push_back(y_point);

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
