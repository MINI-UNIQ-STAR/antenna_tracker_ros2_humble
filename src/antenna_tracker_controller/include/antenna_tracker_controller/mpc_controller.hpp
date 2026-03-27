#ifndef ANTENNA_TRACKER_CONTROLLER__MPC_CONTROLLER_HPP_
#define ANTENNA_TRACKER_CONTROLLER__MPC_CONTROLLER_HPP_

#include "acados_solver_antenna_tracker.h"

namespace antenna_tracker_controller
{

class MpcController
{
public:
  MpcController();
  ~MpcController();

  // Rule of Five: raw pointer ownership — disable copy and move
  MpcController(const MpcController &) = delete;
  MpcController & operator=(const MpcController &) = delete;
  MpcController(MpcController &&) = delete;
  MpcController & operator=(MpcController &&) = delete;

  void init();
  bool is_initialized() const { return acados_capsule_ != nullptr; }

  void compute(
    double az_target, double az_current, double az_vel,
    double el_target, double el_current, double el_vel,
    double & az_output, double & el_output);

  void set_mpc_to_hz_scale(double scale) { mpc_to_hz_scale_ = scale; }

  /// Gravity equilibrium feedforward coefficient (measured: NEMA23 + 20:1 + 500g Yagi @0.9m)
  /// u_el_eq = kGravityCoeff * cos(el)  [rad/s²]
  static constexpr double kGravityCoeff = 10.77;

private:
  antenna_tracker_solver_capsule * acados_capsule_;
  double mpc_to_hz_scale_{1.0};  ///< conversion: solver rad/s² → stepper Hz
};

}  // namespace antenna_tracker_controller

#endif  // ANTENNA_TRACKER_CONTROLLER__MPC_CONTROLLER_HPP_
