#pragma once

#include <cmath>

#include <ocs2_core/automatic_differentiation/Types.h>

#include "opti_pessi_interface/OptiPessiModelParameters.h"
#include "opti_pessi_interface/definitions.h"

namespace opti_pessi {

/**
 * The wrap* overloads below are what let every function in this header serve BOTH scalar_t
 * (double, for the closed-loop simulation) and ad_scalar_t (CppAD::AD<CG<double>>, for the
 * CppAD-generated cost/constraint/dynamics libraries) from a single implementation. Calling
 * std::cos on an ad_scalar_t, or assigning a double to one, is exactly the class of error that
 * sinks hand-written AD code.
 */
inline scalar_t wrapCos(scalar_t x) { return std::cos(x); }
inline scalar_t wrapSin(scalar_t x) { return std::sin(x); }
inline scalar_t wrapCosh(scalar_t x) { return std::cosh(x); }
inline scalar_t wrapSinh(scalar_t x) { return std::sinh(x); }
inline ocs2::ad_scalar_t wrapCos(const ocs2::ad_scalar_t& x) { return CppAD::cos(x); }
inline ocs2::ad_scalar_t wrapSin(const ocs2::ad_scalar_t& x) { return CppAD::sin(x); }
inline ocs2::ad_scalar_t wrapCosh(const ocs2::ad_scalar_t& x) { return CppAD::cosh(x); }
inline ocs2::ad_scalar_t wrapSinh(const ocs2::ad_scalar_t& x) { return CppAD::sinh(x); }

/** R01(theta) * v, i.e. R(-theta) * v : world -> body. */
template <typename Scalar, typename Derived>
Eigen::Matrix<Scalar, 2, 1> applyR01(const Scalar& theta, const Eigen::MatrixBase<Derived>& v) {
  const Scalar c = wrapCos(theta);
  const Scalar s = wrapSin(theta);
  return Eigen::Matrix<Scalar, 2, 1>(c * v(0) + s * v(1), -s * v(0) + c * v(1));
}

/** R(theta) * v : body -> world. */
template <typename Scalar, typename Derived>
Eigen::Matrix<Scalar, 2, 1> applyR(const Scalar& theta, const Eigen::MatrixBase<Derived>& v) {
  const Scalar c = wrapCos(theta);
  const Scalar s = wrapSin(theta);
  return Eigen::Matrix<Scalar, 2, 1>(c * v(0) - s * v(1), s * v(0) + c * v(1));
}

/** Centre of pressure on the support segment: z = p0 + alpha * (p1 - p0). */
template <typename Scalar>
Eigen::Matrix<Scalar, 2, 1> computeCop(const Eigen::Matrix<Scalar, 2, 1>& p0, const Eigen::Matrix<Scalar, 2, 1>& p1,
                                       const Scalar& alpha) {
  return p0 + alpha * (p1 - p0);
}

/**
 * Splits the horizontal force m * ddc implied by the LIP between the two stance feet using
 * (beta, gamma), with ddc = omega^2 * (c - z).
 */
template <typename Scalar>
void computeTangentialForces(const Eigen::Matrix<Scalar, 2, 1>& c, const Eigen::Matrix<Scalar, 2, 1>& p0,
                             const Eigen::Matrix<Scalar, 2, 1>& p1, const Scalar& alpha, const Scalar& beta, const Scalar& gamma,
                             const Scalar& w, const Scalar& mass, Eigen::Matrix<Scalar, 2, 1>& f0, Eigen::Matrix<Scalar, 2, 1>& f1) {
  const auto cop = computeCop(p0, p1, alpha);
  const Eigen::Matrix<Scalar, 2, 1> ddc = (w * w) * (c - cop);
  f0(0) = beta * mass * ddc(0);
  f0(1) = gamma * mass * ddc(1);
  f1(0) = (Scalar(1) - beta) * mass * ddc(0);
  f1(1) = (Scalar(1) - gamma) * mass * ddc(1);
}

/** Vertical moment about the CoM produced by the two tangential foot forces. */
template <typename Scalar>
Scalar yawTorque(const Eigen::Matrix<Scalar, 2, 1>& c, const Eigen::Matrix<Scalar, 2, 1>& p0, const Eigen::Matrix<Scalar, 2, 1>& p1,
                 const Eigen::Matrix<Scalar, 2, 1>& f0, const Eigen::Matrix<Scalar, 2, 1>& f1) {
  return (p0(0) - c(0)) * f0(1) - (p0(1) - c(1)) * f0(0) + (p1(0) - c(0)) * f1(1) - (p1(1) - c(1)) * f1(0);
}

/**
 * Exact discrete LIP step over one contact phase of duration u(RobotU::DT): the closed-form flow
 * of ddc = omega^2 (c - z) with constant CoP, plus a forward-Euler step for yaw. Maps
 * (x in R^10, u in R^8) to x_next in R^10.
 */
template <typename Vec>
Vec lipMap(const Vec& x, const Vec& u, typename Vec::Scalar w, typename Vec::Scalar mass, typename Vec::Scalar inertia) {
  using Scalar = typename Vec::Scalar;
  Vec xNext(RobotX::DIM);
  const Eigen::Matrix<Scalar, 2, 1> c(x(RobotX::CX), x(RobotX::CY));
  const Scalar theta = x(RobotX::TH);
  const Eigen::Matrix<Scalar, 2, 1> dc(x(RobotX::DCX), x(RobotX::DCY));
  const Scalar dtheta = x(RobotX::DTH);
  const Eigen::Matrix<Scalar, 2, 1> p0(x(RobotX::P0X), x(RobotX::P0Y));
  const Eigen::Matrix<Scalar, 2, 1> p1(x(RobotX::P1X), x(RobotX::P1Y));
  const Eigen::Matrix<Scalar, 2, 1> p0n(u(RobotU::P0X), u(RobotU::P0Y));
  const Eigen::Matrix<Scalar, 2, 1> p1n(u(RobotU::P1X), u(RobotU::P1Y));
  const Scalar alpha = u(RobotU::ALPHA);
  const Scalar dt = u(RobotU::DT);
  const Scalar beta = u(RobotU::BETA);
  const Scalar gamma = u(RobotU::GAMMA);

  const Scalar ch = wrapCosh(w * dt);
  const Scalar sh = wrapSinh(w * dt);
  const auto cop = computeCop(p0, p1, alpha);
  Eigen::Matrix<Scalar, 2, 1> f0, f1;
  computeTangentialForces(c, p0, p1, alpha, beta, gamma, w, mass, f0, f1);
  const Scalar tau = yawTorque(c, p0, p1, f0, f1);

  const Eigen::Matrix<Scalar, 2, 1> cNext = ch * c + (sh / w) * dc + (Scalar(1) - ch) * cop;
  const Scalar thetaNext = theta + dt * dtheta;
  const Eigen::Matrix<Scalar, 2, 1> dcNext = (w * sh) * c + ch * dc - (w * sh) * cop;
  const Scalar dthetaNext = dtheta + dt * tau / inertia;

  xNext(RobotX::CX) = cNext(0);
  xNext(RobotX::CY) = cNext(1);
  xNext(RobotX::TH) = thetaNext;
  xNext(RobotX::DCX) = dcNext(0);
  xNext(RobotX::DCY) = dcNext(1);
  xNext(RobotX::DTH) = dthetaNext;
  xNext(RobotX::P0X) = p0n(0);
  xNext(RobotX::P0Y) = p0n(1);
  xNext(RobotX::P1X) = p1n(0);
  xNext(RobotX::P1Y) = p1n(1);
  // This phase's stance feet become the next phase's "previous" feet -- a pure copy.
  xNext(RobotX::PP0X) = p0(0);
  xNext(RobotX::PP0Y) = p0(1);
  xNext(RobotX::PP1X) = p1(0);
  xNext(RobotX::PP1Y) = p1(1);
  return xNext;
}

/** Non-template entry point to lipMap for the closed-loop simulation and tests. */
vector_t lipMapScalar(const vector_t& x, const vector_t& u, scalar_t w, scalar_t mass, scalar_t inertia);

/**
 * State-dependent part of the running cost, shared by the stage and the final cost.
 *
 * The wtheta term is algebraically identically zero (it is the heading-alignment term of the
 * Python reference, where wtheta = 0). It is kept so this port stays a 1:1 image of the original.
 */
template <typename Scalar>
Scalar runningStateCost(const Eigen::Matrix<Scalar, 2, 1>& c, const Scalar& theta, const Eigen::Matrix<Scalar, 2, 1>& dc,
                        const Scalar& dtheta, const Eigen::Matrix<Scalar, 2, 1>& cGoal, const OptiPessiModelParameters& params) {
  const Eigen::Matrix<Scalar, 2, 1> e = c - cGoal;
  Scalar cost = Scalar(params.wc) * e.dot(e);
  cost += Scalar(params.wdc) * dc.dot(dc);
  cost += Scalar(params.wdtheta) * dtheta * dtheta;
  const Scalar dc2 = dc.dot(dc);
  const Scalar ct = wrapCos(theta);
  const Scalar st = wrapSin(theta);
  const Scalar align = dc2 * ct * ct - dc(0) * dc(0) + dc2 * st * st - dc(1) * dc(1);
  cost += Scalar(params.wtheta) * align * align;
  return cost;
}

}  // namespace opti_pessi
