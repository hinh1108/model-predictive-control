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
#include "json.hpp"

// for convenience
using json = nlohmann::json;
using Eigen::VectorXd;

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

static const double Lf = 2.67;

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
double polyeval(VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
VectorXd polyfit(VectorXd xvals, VectorXd yvals,
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

    if (sdata.size() > 2 && sdata[0] == '4' && sdata[1] == '2') {
      string s = hasData(sdata);
      if (s != "") {
        auto j = json::parse(s);
        string event = j[0].get<string>();
        if (event == "telemetry") {
          // j[1] is the data JSON object
          vector<double> ptsx = j[1]["ptsx"];
          vector<double> ptsy = j[1]["ptsy"];
          const double px = j[1]["x"];
          const double py = j[1]["y"];
          const double psi = j[1]["psi"];
          const double v = j[1]["speed"];
          const double delta = j[1]["steering_angle"];
          const double accel = j[1]["throttle"];
          const size_t n_points = ptsx.size();

          VectorXd waypoints_x(n_points);
          VectorXd waypoints_y(n_points);
          vector<double> next_x_vals;
          vector<double> next_y_vals;
          const double cos_psi = cos(-psi);
          const double sin_psi = sin(-psi);

          // transform waypoints to local car coords
          for (int t = 0; t < n_points; ++t) {
            const double dx = ptsx[t] - px;
            const double dy = ptsy[t] - py;
            const double next_x = dx * cos_psi - dy * sin_psi;
            const double next_y = dy * cos_psi + dx * sin_psi;

            waypoints_x[t] = next_x;
            waypoints_y[t] = next_y;

            // store waypoints for display on simulator
            next_x_vals.push_back(next_x);
            next_y_vals.push_back(next_y);
          }

          // fit curve (2nd degree polynomial) to transformed waypoints
          VectorXd coeffs = polyfit(waypoints_x, waypoints_y, 2);
          const double cte  = coeffs[0];         // cross track error
          const double epsi = -atan(coeffs[1]);  // orientation error

          // predict vehicle expected state at current time + latency
          const double latency = 0.1;
          const double px_pred   = v * latency;
          const double py_pred   = 0;
          const double psi_pred  = -v * delta * latency / Lf;
          const double v_pred    = v + accel * latency;
          const double cte_pred  = cte + v * sin(epsi) * latency;
          const double epsi_pred = epsi + psi_pred;

          VectorXd state(6);
          state << px_pred,
                   py_pred,
                   psi_pred,
                   v_pred,
                   cte_pred,
                   epsi_pred;
          vector<double> mpc_points = mpc.solve(state, coeffs);
          const int N = (mpc_points.size() - 2) / 2;

          // normalize steering angle to between [-1..1]
          const double steer_value    = mpc_points[0] / deg2rad(25);
          const double throttle_value = mpc_points[1];

          json msgJson;
          msgJson["steering_angle"] = -steer_value;
          msgJson["throttle"]       =  throttle_value;

          // Display the MPC predicted trajectory
          vector<double> mpc_x_vals;
          vector<double> mpc_y_vals;
          for (size_t t = 2; t < N; ++t) {
            mpc_x_vals.push_back(mpc_points[t]);
            mpc_y_vals.push_back(mpc_points[t + N]);
          }

          // simulator green line
          msgJson["mpc_x"] = mpc_x_vals;
          msgJson["mpc_y"] = mpc_y_vals;

          // simulator yellow line
          msgJson["next_x"] = next_x_vals;
          msgJson["next_y"] = next_y_vals;

          auto msg = "42[\"steer\"," + msgJson.dump() + "]";

          // Latency
          // The purpose is to mimic real driving conditions where
          // the car does not actuate the commands instantly.
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
                     size_t, size_t) {});

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
