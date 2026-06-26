#include "opti_pessi_control/OptiPessiAugmentedCost.h"

namespace ocs2 {
namespace quadruped {

OptiPessiAugmentedCost::OptiPessiAugmentedCost() {
    Q_.setIdentity(10, 10);
    Q_.diagonal() << 10.0, 10.0,  // Posizione CoM (c_x, c_y)
                     5.0,         // Orientamento tronco (theta)
                     50.0, 50.0,  // Velocità CoM (c_dot_x, c_dot_y)
                     10.0,        // Velocità angolare (theta_dot)
                     2.0, 2.0,    // Posizione piede front (p0_x, p0_y)
                     2.0, 2.0;    // Posizione piede rear (p1_x, p1_y)

    R_.setIdentity(8, 8);
    R_.diagonal() << 1.0, 1.0,    
                     1.0, 1.0,    
                     0.1,         
                     0.01, 0.01,  
                     10.0;        
}

OptiPessiAugmentedCost* OptiPessiAugmentedCost::clone() const {
    return new OptiPessiAugmentedCost(*this);
}

scalar_t OptiPessiAugmentedCost::getValue(scalar_t t, const vector_t& x, const vector_t& u, 
                                      const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
    
    vector_t x_nominal_des = targetTrajectories.getDesiredState(t);
    vector_t u_nominal_des = targetTrajectories.getDesiredInput(t).head(8);

    vector_t x_op = x.head(10);
    vector_t x_pe = x.tail(10);
    vector_t u_op = u.head(8);
    vector_t u_pe = u.tail(8);

    vector_t x_error_op = x_op - x_nominal_des.head(10);
    vector_t x_error_pe = x_pe - x_nominal_des.tail(10);
    
    scalar_t cost = 0.5 * x_error_op.dot(Q_ * x_error_op) + 0.5 * x_error_pe.dot(Q_ * x_error_pe);

    double s_x = 0.1934; 
    double wp = 1.0;     
    
    // Optimistic model input cost
    double e0_x = u_op[0] - x_op[0] - s_x;
    double e0_y = u_op[1] - x_op[1];
    double e1_x = u_op[2] - x_op[0] + s_x;
    double e1_y = u_op[3] - x_op[1];
    cost += 0.5 * wp * (e0_x * e0_x + e0_y * e0_y + e1_x * e1_x + e1_y * e1_y);

    cost += 0.5 * R_(4, 4) * std::pow(u_op[4] - 0.5, 2);               
    cost += 0.5 * R_(5, 5) * std::pow(u_op[5] - u_nominal_des[5], 2);  
    cost += 0.5 * R_(6, 6) * std::pow(u_op[6] - u_nominal_des[6], 2);  
    cost += 0.5 * R_(7, 7) * std::pow(u_op[7] - 0.35, 2);              

    // Pessimistic model input cost
    double e0_pe_x = u_pe[0] - x_pe[0] - s_x;
    double e0_pe_y = u_pe[1] - x_pe[1];
    double e1_pe_x = u_pe[2] - x_pe[0] + s_x;
    double e1_pe_y = u_pe[3] - x_pe[1];
    cost += 0.5 * wp * (e0_pe_x * e0_pe_x + e0_pe_y * e0_pe_y + e1_pe_x * e1_pe_x + e1_pe_y * e1_pe_y);

    cost += 0.5 * R_(4, 4) * std::pow(u_pe[4] - 0.5, 2);
    cost += 0.5 * R_(5, 5) * std::pow(u_pe[5] - u_nominal_des[5], 2);
    cost += 0.5 * R_(6, 6) * std::pow(u_pe[6] - u_nominal_des[6], 2);
    cost += 0.5 * R_(7, 7) * std::pow(u_pe[7] - 0.35, 2);

    return cost;
}

ScalarFunctionQuadraticApproximation OptiPessiAugmentedCost::getQuadraticApproximation(
    scalar_t t, const vector_t& x, const vector_t& u, 
    const TargetTrajectories& targetTrajectories, const PreComputation& preComp) const {
    
    ScalarFunctionQuadraticApproximation alloc;
    alloc.f = getValue(t, x, u, targetTrajectories, preComp);

    vector_t x_nominal_des = targetTrajectories.getDesiredState(t);
    vector_t u_nominal_des = targetTrajectories.getDesiredInput(t).head(8);
    vector_t x_op = x.head(10);
    vector_t x_pe = x.tail(10);
    vector_t u_op = u.head(8);
    vector_t u_pe = u.tail(8);

    vector_t x_error_op = x_op - x_nominal_des.head(10);
    vector_t x_error_pe = x_pe - x_nominal_des.tail(10);

    alloc.dfdx = vector_t::Zero(20);
    alloc.dfdu = vector_t::Zero(16);
    alloc.dfdxx = matrix_t::Zero(20, 20);
    alloc.dfduu = matrix_t::Zero(16, 16);
    alloc.dfdux = matrix_t::Zero(16, 20);

    alloc.dfdx.head(10) = Q_ * x_error_op;
    alloc.dfdx.tail(10) = Q_ * x_error_pe;

    alloc.dfdxx.topLeftCorner(10, 10) = Q_;
    alloc.dfdxx.bottomRightCorner(10, 10) = Q_;

    double s_x = 0.1934;
    double wp = 1.0;

    // Optimistic gradients and Hessians
    alloc.dfdu[0] = wp * (u_op[0] - x_op[0] - s_x);
    alloc.dfdu[1] = wp * (u_op[1] - x_op[1]);
    alloc.dfdu[2] = wp * (u_op[2] - x_op[0] + s_x);
    alloc.dfdu[3] = wp * (u_op[3] - x_op[1]);

    alloc.dfdx[0] += -wp * (u_op[0] - x_op[0] - s_x) - wp * (u_op[2] - x_op[0] + s_x);
    alloc.dfdx[1] += -wp * (u_op[1] - x_op[1]) - wp * (u_op[3] - x_op[1]);

    alloc.dfduu(0, 0) = wp; alloc.dfduu(1, 1) = wp;
    alloc.dfduu(2, 2) = wp; alloc.dfduu(3, 3) = wp;
    alloc.dfdxx(0, 0) += 2.0 * wp; alloc.dfdxx(1, 1) += 2.0 * wp;

    alloc.dfdux(0, 0) = -wp; alloc.dfdux(1, 1) = -wp;
    alloc.dfdux(2, 0) = -wp; alloc.dfdux(3, 1) = -wp;

    alloc.dfdu[4] = R_(4, 4) * (u_op[4] - 0.5);
    alloc.dfdu[5] = R_(5, 5) * (u_op[5] - u_nominal_des[5]);
    alloc.dfdu[6] = R_(6, 6) * (u_op[6] - u_nominal_des[6]);
    alloc.dfdu[7] = R_(7, 7) * (u_op[7] - 0.35);

    alloc.dfduu(4, 4) = R_(4, 4);
    alloc.dfduu(5, 5) = R_(5, 5);
    alloc.dfduu(6, 6) = R_(6, 6);
    alloc.dfduu(7, 7) = R_(7, 7);

    // Pessimistic gradients and Hessians
    alloc.dfdu[8] = wp * (u_pe[0] - x_pe[0] - s_x);
    alloc.dfdu[9] = wp * (u_pe[1] - x_pe[1]);
    alloc.dfdu[10] = wp * (u_pe[2] - x_pe[0] + s_x);
    alloc.dfdu[11] = wp * (u_pe[3] - x_pe[1]);

    alloc.dfdx[10] += -wp * (u_pe[0] - x_pe[0] - s_x) - wp * (u_pe[2] - x_pe[0] + s_x);
    alloc.dfdx[11] += -wp * (u_pe[1] - x_pe[1]) - wp * (u_pe[3] - x_pe[1]);

    alloc.dfduu(8, 8) = wp; alloc.dfduu(9, 9) = wp;
    alloc.dfduu(10, 10) = wp; alloc.dfduu(11, 11) = wp;
    alloc.dfdxx(10, 10) += 2.0 * wp; alloc.dfdxx(11, 11) += 2.0 * wp;

    alloc.dfdux(8, 10) = -wp; alloc.dfdux(9, 11) = -wp;
    alloc.dfdux(10, 10) = -wp; alloc.dfdux(11, 11) = -wp;

    alloc.dfdu[12] = R_(4, 4) * (u_pe[4] - 0.5);
    alloc.dfdu[13] = R_(5, 5) * (u_pe[5] - u_nominal_des[5]);
    alloc.dfdu[14] = R_(6, 6) * (u_pe[6] - u_nominal_des[6]);
    alloc.dfdu[15] = R_(7, 7) * (u_pe[7] - 0.35);

    alloc.dfduu(12, 12) = R_(4, 4);
    alloc.dfduu(13, 13) = R_(5, 5);
    alloc.dfduu(14, 14) = R_(6, 6);
    alloc.dfduu(15, 15) = R_(7, 7);

    return alloc;
}

} // namespace quadruped
} // namespace ocs2