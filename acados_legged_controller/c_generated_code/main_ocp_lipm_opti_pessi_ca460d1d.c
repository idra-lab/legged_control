/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */


// standard
#include <stdio.h>
#include <stdlib.h>
// acados
#include "acados/utils/print.h"
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"
#include "acados_solver_ocp_lipm_opti_pessi_ca460d1d.h"

// blasfeo
#include "blasfeo_d_aux_ext_dep.h"

#define NX     OCP_LIPM_OPTI_PESSI_CA460D1D_NX
#define NP     OCP_LIPM_OPTI_PESSI_CA460D1D_NP
#define NU     OCP_LIPM_OPTI_PESSI_CA460D1D_NU
#define NBX0   OCP_LIPM_OPTI_PESSI_CA460D1D_NBX0
#define NP_GLOBAL   OCP_LIPM_OPTI_PESSI_CA460D1D_NP_GLOBAL


int main()
{

    ocp_lipm_opti_pessi_ca460d1d_solver_capsule *acados_ocp_capsule = ocp_lipm_opti_pessi_ca460d1d_acados_create_capsule();
    // there is an opportunity to change the number of shooting intervals in C without new code generation
    int N = OCP_LIPM_OPTI_PESSI_CA460D1D_N;
    // allocate the array and fill it accordingly
    double* new_time_steps = NULL;
    int status = ocp_lipm_opti_pessi_ca460d1d_acados_create_with_discretization(acados_ocp_capsule, N, new_time_steps);

    if (status)
    {
        printf("ocp_lipm_opti_pessi_ca460d1d_acados_create() returned status %d. Exiting.\n", status);
        exit(1);
    }

    ocp_nlp_config *nlp_config = ocp_lipm_opti_pessi_ca460d1d_acados_get_nlp_config(acados_ocp_capsule);
    ocp_nlp_dims *nlp_dims = ocp_lipm_opti_pessi_ca460d1d_acados_get_nlp_dims(acados_ocp_capsule);
    ocp_nlp_in *nlp_in = ocp_lipm_opti_pessi_ca460d1d_acados_get_nlp_in(acados_ocp_capsule);
    ocp_nlp_out *nlp_out = ocp_lipm_opti_pessi_ca460d1d_acados_get_nlp_out(acados_ocp_capsule);
    ocp_nlp_solver *nlp_solver = ocp_lipm_opti_pessi_ca460d1d_acados_get_nlp_solver(acados_ocp_capsule);
    void *nlp_opts = ocp_lipm_opti_pessi_ca460d1d_acados_get_nlp_opts(acados_ocp_capsule);
    // initial condition
    double lbx0[NBX0];
    double ubx0[NBX0];
    lbx0[0] = -0.8;
    ubx0[0] = -0.8;
    lbx0[1] = -0.5;
    ubx0[1] = -0.5;
    lbx0[2] = 0;
    ubx0[2] = 0;
    lbx0[3] = 0;
    ubx0[3] = 0;
    lbx0[4] = 0;
    ubx0[4] = 0;
    lbx0[5] = 0;
    ubx0[5] = 0;
    lbx0[6] = -0.5593;
    ubx0[6] = -0.5593;
    lbx0[7] = -0.634;
    ubx0[7] = -0.634;
    lbx0[8] = -1.0407;
    ubx0[8] = -1.0407;
    lbx0[9] = -0.366;
    ubx0[9] = -0.366;
    lbx0[10] = 0;
    ubx0[10] = 0;
    lbx0[11] = -0.8;
    ubx0[11] = -0.8;
    lbx0[12] = -0.5;
    ubx0[12] = -0.5;
    lbx0[13] = 0;
    ubx0[13] = 0;
    lbx0[14] = 0;
    ubx0[14] = 0;
    lbx0[15] = 0;
    ubx0[15] = 0;
    lbx0[16] = 0;
    ubx0[16] = 0;
    lbx0[17] = -0.5593;
    ubx0[17] = -0.5593;
    lbx0[18] = -0.634;
    ubx0[18] = -0.634;
    lbx0[19] = -1.0407;
    ubx0[19] = -1.0407;
    lbx0[20] = -0.366;
    ubx0[20] = -0.366;
    lbx0[21] = 0;
    ubx0[21] = 0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);

    // initialization for state values
    double x_init[NX];
    x_init[0] = 0.0;
    x_init[1] = 0.0;
    x_init[2] = 0.0;
    x_init[3] = 0.0;
    x_init[4] = 0.0;
    x_init[5] = 0.0;
    x_init[6] = 0.0;
    x_init[7] = 0.0;
    x_init[8] = 0.0;
    x_init[9] = 0.0;
    x_init[10] = 0.0;
    x_init[11] = 0.0;
    x_init[12] = 0.0;
    x_init[13] = 0.0;
    x_init[14] = 0.0;
    x_init[15] = 0.0;
    x_init[16] = 0.0;
    x_init[17] = 0.0;
    x_init[18] = 0.0;
    x_init[19] = 0.0;
    x_init[20] = 0.0;
    x_init[21] = 0.0;

    // initial value for control input
    double u0[NU];
    u0[0] = 0.0;
    u0[1] = 0.0;
    u0[2] = 0.0;
    u0[3] = 0.0;
    u0[4] = 0.0;
    u0[5] = 0.0;
    u0[6] = 0.0;
    u0[7] = 0.0;
    u0[8] = 0.0;
    u0[9] = 0.0;
    u0[10] = 0.0;
    u0[11] = 0.0;
    u0[12] = 0.0;
    u0[13] = 0.0;
    u0[14] = 0.0;
    u0[15] = 0.0;
    u0[16] = 0.0;
    u0[17] = 0.0;
    u0[18] = 0.0;
    u0[19] = 0.0;
    u0[20] = 0.0;
    u0[21] = 0.0;

    // prepare evaluation
    int NTIMINGS = 1;
    double min_time = 1e12;
    double kkt_norm_inf;
    double elapsed_time;
    int sqp_iter;

    double xtraj[NX * (N+1)];
    double utraj[NU * N];

    // solve ocp in loop
    for (int ii = 0; ii < NTIMINGS; ii++)
    {
        // initialize solution
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x_init);
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
        }
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x_init);
        status = ocp_lipm_opti_pessi_ca460d1d_acados_solve(acados_ocp_capsule);
        ocp_nlp_get(nlp_solver, "time_tot", &elapsed_time);
        min_time = MIN(elapsed_time, min_time);
    }

    /* print solution and statistics */
    for (int ii = 0; ii <= nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "x", &xtraj[ii*NX]);
    for (int ii = 0; ii < nlp_dims->N; ii++)
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, ii, "u", &utraj[ii*NU]);

    printf("\n--- xtraj ---\n");
    d_print_exp_tran_mat( NX, N+1, xtraj, NX);
    printf("\n--- utraj ---\n");
    d_print_exp_tran_mat( NU, N, utraj, NU );
    // print_ocp_nlp_out(nlp_solver->dims, nlp_out);

    printf("\nsolved ocp %d times, solution printed above\n\n", NTIMINGS);

    if (status == ACADOS_SUCCESS)
    {
        printf("ocp_lipm_opti_pessi_ca460d1d_acados_solve(): SUCCESS!\n");
    }
    else
    {
        printf("ocp_lipm_opti_pessi_ca460d1d_acados_solve() failed with status %d.\n", status);
    }

    // get solution
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "kkt_norm_inf", &kkt_norm_inf);
    ocp_nlp_get(nlp_solver, "sqp_iter", &sqp_iter);

    ocp_lipm_opti_pessi_ca460d1d_acados_print_stats(acados_ocp_capsule);

    printf("\nSolver info:\n");
    printf(" SQP iterations %2d\n minimum time for %d solve %f [ms]\n KKT %e\n",
           sqp_iter, NTIMINGS, min_time*1000, kkt_norm_inf);



    // free solver
    status = ocp_lipm_opti_pessi_ca460d1d_acados_free(acados_ocp_capsule);
    if (status) {
        printf("ocp_lipm_opti_pessi_ca460d1d_acados_free() returned status %d. \n", status);
    }
    // free solver capsule
    status = ocp_lipm_opti_pessi_ca460d1d_acados_free_capsule(acados_ocp_capsule);
    if (status) {
        printf("ocp_lipm_opti_pessi_ca460d1d_acados_free_capsule() returned status %d. \n", status);
    }

    return status;
}
