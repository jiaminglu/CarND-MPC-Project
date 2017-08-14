#ifndef MPC_H
#define MPC_H

#include <vector>
#include "Eigen-3.3/Eigen/Core"

using namespace std;

//timestep length and duration
const size_t N = 10;
const double dt = 0.05;

//latency in ms
const int latency = 100;

//number of control input from mpc prediction considering latency
const int lat_idx = double(latency) / 1000 / dt;

//control targets
//const double ref_cte = 0;
//const double ref_epsi = 0;
const double ref_v = 75;

//MPC parameters
const double coeff_cte = 2000;
const double coeff_epsi = 2000;
const double coeff_v = 1;
const double coeff_delta = 100;
const double coeff_a = 10;
const double coeff_d_delta = 100;
const double coeff_d_a = 10;
//The upper and lower limits of delta
const double constr_delta = 0.436332;
// Acceleration/decceleration upper and lower limits.
const double constr_a = 1.0;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;

// The solver takes all the state variables and actuator
// variables in a singular vector. Thus, we should to establish
// when one variable starts and another ends to make our lifes easier.
size_t x_start = 0;
size_t y_start = x_start + N;
size_t psi_start = y_start + N;
size_t v_start = psi_start + N;
size_t cte_start = v_start + N;
size_t epsi_start = cte_start + N;
size_t delta_start = epsi_start + N;
size_t a_start = delta_start + N - 1;

struct Prediction {
    vector<double> x;
    vector<double> y;
    vector<double> delta;
    vector<double> a;
};

class MPC {
 public:
  MPC();

  virtual ~MPC();

  // Solve the model given an initial state and polynomial coefficients.
  // Return the first actuatotions.
  Prediction Solve(Eigen::VectorXd state, Eigen::VectorXd coeffs);
};

#endif /* MPC_H */
