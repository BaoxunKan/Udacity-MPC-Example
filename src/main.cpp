#include <math.h>
#include <uWS/uWS.h>

#include <chrono>
#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "helpers.h"
#include "json.hpp"

// for convenience
using nlohmann::json;
using std::string;
using std::vector;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

int main(int argc, char *argv[]) {
  uWS::Hub h;

  // MPC is initialized here!
  const string config_path = argc > 1 ? argv[1] : "mpc_config.json";
  MPC mpc(config_path);

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    std::cout << sdata << std::endl;
    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          double px = j[1]["x"];
          double py = j[1]["y"];
          double psi = j[1]["psi"];
          double v = j[1]["speed"];
          v *= 0.44704;  // Convert from mph to m/s
          const double steer_sim = j[1]["steering_angle"];
          const double throttle_sim = j[1]["throttle"];

          /**
           * Predict the state 100ms ahead so the solver sees where the car
           * will be when this command actually takes effect.
           */
          const double latency = 0.1;
          const double Lf = 2.67;
          px += v * cos(psi) * latency;
          py += v * sin(psi) * latency;
          // Simulator steering has the opposite sign of the MPC delta.
          psi += v / Lf * (-steer_sim) * latency;
          v += throttle_sim * latency;

          /**
           * TODO: Calculate steering angle and throttle using MPC.
           * Both are in between [-1, 1].
           */

          auto ptsx_vehicle = Eigen::VectorXd(ptsx.size());
          auto ptsy_vehicle = Eigen::VectorXd(ptsy.size());
          // Rotation matrix is given by:
          // | cos(psi) -sin(psi) |
          // | sin(psi) cos(psi) |
          // The rotation matrix is used to transform the waypoints from the
          // global coordinate system to the vehicle coordinate system.
          for (std::size_t i = 0; i < ptsx.size(); ++i) {
            ptsx_vehicle(i) =
                cos(psi) * (ptsx[i] - px) + sin(psi) * (ptsy[i] - py);
            ptsy_vehicle(i) =
                -sin(psi) * (ptsx[i] - px) + cos(psi) * (ptsy[i] - py);
          }

          // fit a polynomial to the waypoints in the vehicle coordinate system
          auto coeffs = polyfit(ptsx_vehicle, ptsy_vehicle, 3);
          // evaluate the polynomial at the current x position
          double cte = polyeval(coeffs, 0);
          double epsi = -atan(coeffs[1]);

          double x_delay = v * latency;
          double y_delay = 0;
          double psi_delay = v / Lf * (-steer_sim) * latency;
          double v_delay = v + throttle_sim * latency;
          double cte_delay = cte + v * sin(epsi) * latency;
          double epsi_delay = epsi + v / Lf * (-steer_sim) * latency;

          Eigen::VectorXd state(6);
          state << x_delay, y_delay, psi_delay, v_delay, cte_delay, epsi_delay;

          auto solution = mpc.Solve(state, coeffs);

          double steer_value = -solution[0] / deg2rad(25);
          double throttle_value = solution[1];

          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the
          //   steering value back. Otherwise the values will be in between
          //   [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          msgJson["steering_angle"] = steer_value;
          msgJson["throttle"] = throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Green line
           */
          for (int i = 2; i < solution.size(); i += 2) {
            mpc_x_vals.push_back(solution[i]);
            mpc_y_vals.push_back(solution[i + 1]);
          }

          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line
          vector<double> next_x_vals(ptsx_vehicle.data(),
                                     ptsx_vehicle.data() + ptsx_vehicle.size());
          vector<double> next_y_vals(ptsy_vehicle.data(),
                                     ptsy_vehicle.data() + ptsy_vehicle.size());

          /**
           * TODO: add (x,y) points to list here, points are in reference to
           *   the vehicle's coordinate system the points in the simulator are
           *   connected by a Yellow line
           */
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          //   the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          //   around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE SUBMITTING.
          std::this_thread::sleep_for(std::chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }  // end "telemetry" if
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }  // end websocket if
  });  // end h.onMessage

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