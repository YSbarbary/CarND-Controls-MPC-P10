#include <math.h>
#include <uWS/uWS.h>
#include <chrono>
#include <iostream>
#include <thread>
#include <vector>
#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include "MPC.h"
#include "json.hpp"

// for convenience
using json = nlohmann::json;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Checks if the SocketIO event has JSON data.
// If there is data the JSON object in string format will be returned,
// else the empty string "" will be returned.
string hasData(string s) {
  auto found_null = s.find("null");
  auto b1 = s.find_first_of("[");
  auto b2 = s.rfind("}]");
  if (found_null != string::npos) {
    return "";
  } else if (b1 != string::npos && b2 != string::npos) {
    return s.substr(b1, b2 - b1 + 2);
  }
  return "";
}

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

int main() {
  uWS::Hub h;

  // MPC is initialized here!
  MPC mpc;

  h.onMessage([&mpc](uWS::WebSocket<uWS::SERVER> ws, char *data, size_t length,
                     uWS::OpCode opCode) {
    // "42" at the start of the message means there's a websocket message event.
    // The 4 signifies a websocket message
    // The 2 signifies a websocket event
    string sdata = string(data).substr(0, length);
    cout << sdata << endl;
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
          double steer_value = j[1]["steering_angle"];
          double throttle_value = j[1]["throttle"];
          double Lf = 2.67;


          // Predicting state parameters for a latency of 100 ms
          double latency = 0.1;
          px = px + v * cos(psi) * latency;
          py = py + v * sin(psi) * latency;
          psi = psi - v * steer_value / Lf * latency;
          v = v + throttle_value * latency;

          // Coordinate system transformation for the waypoints: rotation from the map coord system to the car's system
          for (size_t i = 0; i < ptsx.size(); i++) {
            double delta_x = ptsx[i] - px;
            double delta_y = ptsy[i] - py;
            ptsx[i] = delta_x * cos(0 - psi) - delta_y * sin(0 - psi);
            ptsy[i] = delta_x * sin(0 - psi) + delta_y * cos(0 - psi);
          }


          // Now I want to fit a polynomial (3rd order is enough) to the waypoints in the car's coord system.
          // But polyfit function requires a VectorXd input, so the vectors <double> are transformed to VectorX first.
          double* ptrx = &ptsx[0];
          double* ptry = &ptsy[0];
          Eigen::Map<Eigen::VectorXd> ptsx_transform(ptrx, 6);
          Eigen::Map<Eigen::VectorXd> ptsy_transform(ptry, 6);

          // And now do the fit:
          // (remember this is the coord. system of the car, i.e. x points ahead, y points to the left)
          auto coeffs = polyfit(ptsx_transform, ptsy_transform, 3);

          // Now we compute the variables we want to be zero so the car's stay at the track.
          // i) cte: Now that the car is in the origin of the coord. system, we just evaluate the fit at x=0.
          // i.e. the cte is along the y-axis only (> 0 to the left, < 0 to the right) which is a good approximation.
          double cte = polyeval(coeffs, 0);
          // ii) epsi: Accordingly psi is now zero at the car's coord system, so the approximation for the psi error is:
          double epsi = -atan(coeffs[1]);

          // px, py, psi are all zero since our coordinate transform
          Eigen::VectorXd state(6);
          state << 0, 0, 0, v, cte, epsi;


          // We pass our state and coefficients of the fit to the MP controller.
          // The MPC selects the trajectory with minimum cost -given the constraints of the model- and deliver us a
          // vector with the corresponding control inputs. The idea is we will apply the first control input
          // (steering angle & throttle) and then repeat the loop.
          auto vars = mpc.Solve(state, coeffs);


          // This is optional, but I'll want to plot the reference path back in the simulator (yellow line)
          // These (x,y) values are in car's reference system.
          vector<double> next_x_vals;
          vector<double> next_y_vals;

          double poly_inc = 2.5;
          int num_points = 25;

          for (int i = 1; i < num_points; i++) {
            next_x_vals.push_back(poly_inc * i);
            next_y_vals.push_back(polyeval(coeffs, poly_inc * i));
          }

          // This is optional, but I'll want to plot the MPC trajectory back in the simulator (green line)
          // These (x,y) values are in car's reference system.
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;

          for (size_t i = 2; i < vars.size(); i++) {
            if (i%2 == 0) {
              mpc_x_vals.push_back(vars[i]);
            } else {
              mpc_y_vals.push_back(vars[i]);
            }
          }

          // Back to the server! (send back to the simulator, I mean)
          json msgJson;
          // NOTE: Remember to divide by deg2rad(25) before you send the steering value back.
          // Otherwise the values will be in between [-deg2rad(25), deg2rad(25] instead of [-1, 1].
          // Control inputs:
          msgJson["steering_angle"] = vars[0] / (deg2rad(25) * Lf);
          msgJson["throttle"] = vars[1];

          // Display the MPC predicted trajectory (optional)
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // Display the waypoints/reference line (optional)
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;


          auto msg = "42[\"steer\"," + msgJson.dump() + "]";
          std::cout << msg << std::endl;
          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does actuate the commands instantly.
          //
          // Feel free to play around with this value but should be to drive
          // around the track with 100ms latency.
          //
          // NOTE: REMEMBER TO SET THIS TO 100 MILLISECONDS BEFORE
          // SUBMITTING.
          this_thread::sleep_for(chrono::milliseconds(100));
          ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
        }
      } else {
        // Manual driving
        std::string msg = "42[\"manual\",{}]";
        ws.send(msg.data(), msg.length(), uWS::OpCode::TEXT);
      }
    }
  });

  // We don't need this since we're not using HTTP but if it's removed the
  // program
  // doesn't compile :-(
  h.onHttpRequest([](uWS::HttpResponse *res, uWS::HttpRequest req, char *data,
                     size_t, size_t) {
    const std::string s = "<h1>Hello world!</h1>";
    if (req.getUrl().valueLength == 1) {
      res->end(s.data(), s.length());
    } else {
      // i guess this should be done more gracefully?
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