#include "opti_pessi_control/OptiPessiAugmentedDynamics.h"

namespace ocs2 {
namespace quadruped {

OptiPessiAugmentedDynamics* OptiPessiAugmentedDynamics::clone() const {
    return new OptiPessiAugmentedDynamics(*this);
}

vector_t OptiPessiAugmentedDynamics::computeFlowMap(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) {
    vector_t dxdt = vector_t::Zero(20);

    vector_t x_op = x.head(10); 
    vector_t u_op = u.head(8);  

    vector_t x_pe = x.tail(10); 
    vector_t u_pe = u.tail(8);  

    dxdt.head(10) = computeSingleLipmDynamics(x_op, u_op);
    dxdt.tail(10) = computeSingleLipmDynamics(x_pe, u_pe);

    return dxdt;
}

VectorFunctionLinearApproximation OptiPessiAugmentedDynamics::linearApproximation(scalar_t t, const vector_t& x, const vector_t& u, const PreComputation& preComp) {
    VectorFunctionLinearApproximation linearApproximation;
    linearApproximation.f = computeFlowMap(t, x, u, preComp);
    linearApproximation.dfdx = matrix_t::Zero(20, 20);
    linearApproximation.dfdu = matrix_t::Zero(20, 16);

    vector_t x_op = x.head(10);
    vector_t u_op = u.head(8);
    matrix_t A_op, B_op;
    computeSingleLipmJacobians(x_op, u_op, A_op, B_op);
    linearApproximation.dfdx.block(0, 0, 10, 10) = A_op;
    linearApproximation.dfdu.block(0, 0, 10, 8) = B_op;

    vector_t x_pe = x.tail(10);
    vector_t u_pe = u.tail(8);
    matrix_t A_pe, B_pe;
    computeSingleLipmJacobians(x_pe, u_pe, A_pe, B_pe);
    linearApproximation.dfdx.block(10, 10, 10, 10) = A_pe;
    linearApproximation.dfdu.block(10, 8, 10, 8) = B_pe;

    return linearApproximation;
}

void OptiPessiAugmentedDynamics::computeSingleLipmJacobians(const vector_t& x, const vector_t& u, matrix_t& A, matrix_t& B) {
    A = matrix_t::Zero(10, 10);
    B = matrix_t::Zero(10, 8);
    
    const scalar_t omega_sq = 9.81 / 0.45;
    scalar_t alpha = u[4];

    A(0, 3) = 1.0;
    A(1, 4) = 1.0;
    A(2, 5) = 1.0;

    A(3, 0) = omega_sq;
    A(4, 1) = omega_sq;
    
    B(3, 0) = -omega_sq * (1.0 - alpha);
    B(3, 2) = -omega_sq * alpha;
    B(3, 4) = -omega_sq * (u[2] - u[0]);

    B(4, 1) = -omega_sq * (1.0 - alpha);
    B(4, 3) = -omega_sq * alpha;
    B(4, 4) = -omega_sq * (u[3] - u[1]);
}

vector_t OptiPessiAugmentedDynamics::computeSingleLipmDynamics(const vector_t& x, const vector_t& u) {
    vector_t dxdt_single = vector_t::Zero(10);
    
    scalar_t theta_dot = x[5];
    const scalar_t omega_sq = 9.81 / 0.45;
    
    scalar_t alpha = u[4];
    
    vector_t p0_u = u.segment(0, 2);
    vector_t p1_u = u.segment(2, 2);
    vector_t u_cop = p0_u + alpha * (p1_u - p0_u); 

    dxdt_single.segment(0, 2) = x.segment(3, 2); 
    dxdt_single[2] = theta_dot;                  
    dxdt_single.segment(3, 2) = omega_sq * (x.segment(0, 2) - u_cop); 

    scalar_t I_z = 0.2; 
    scalar_t sum_torques = 0.0;
    
    dxdt_single[5] = sum_torques / I_z; 

    dxdt_single.segment(6, 2) = vector_t::Zero(2); 
    dxdt_single.segment(8, 2) = vector_t::Zero(2);

    return dxdt_single;
}

} // namespace quadruped
} // namespace ocs2