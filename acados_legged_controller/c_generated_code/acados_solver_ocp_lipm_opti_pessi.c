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
#include <assert.h>
#include <string.h> // memcpy
// acados
// #include "acados/utils/print.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

// example specific

#include "lipm_opti_pessi_model/lipm_opti_pessi_model.h"


#include "lipm_opti_pessi_constraints/lipm_opti_pessi_constraints.h"
#include "lipm_opti_pessi_cost/lipm_opti_pessi_cost.h"



#include "acados_solver_ocp_lipm_opti_pessi.h"

#define NX     OCP_LIPM_OPTI_PESSI_NX
#define NZ     OCP_LIPM_OPTI_PESSI_NZ
#define NU     OCP_LIPM_OPTI_PESSI_NU
#define NP     OCP_LIPM_OPTI_PESSI_NP
#define NP_GLOBAL     OCP_LIPM_OPTI_PESSI_NP_GLOBAL
#define NY0    OCP_LIPM_OPTI_PESSI_NY0
#define NY     OCP_LIPM_OPTI_PESSI_NY
#define NYN    OCP_LIPM_OPTI_PESSI_NYN

#define NBX    OCP_LIPM_OPTI_PESSI_NBX
#define NBX0   OCP_LIPM_OPTI_PESSI_NBX0
#define NBU    OCP_LIPM_OPTI_PESSI_NBU
#define NG     OCP_LIPM_OPTI_PESSI_NG
#define NBXN   OCP_LIPM_OPTI_PESSI_NBXN
#define NGN    OCP_LIPM_OPTI_PESSI_NGN

#define NH     OCP_LIPM_OPTI_PESSI_NH
#define NHN    OCP_LIPM_OPTI_PESSI_NHN
#define NH0    OCP_LIPM_OPTI_PESSI_NH0
#define NPHI   OCP_LIPM_OPTI_PESSI_NPHI
#define NPHIN  OCP_LIPM_OPTI_PESSI_NPHIN
#define NPHI0  OCP_LIPM_OPTI_PESSI_NPHI0
#define NR     OCP_LIPM_OPTI_PESSI_NR

#define NS     OCP_LIPM_OPTI_PESSI_NS
#define NS0    OCP_LIPM_OPTI_PESSI_NS0
#define NSN    OCP_LIPM_OPTI_PESSI_NSN

#define NSBX   OCP_LIPM_OPTI_PESSI_NSBX
#define NSBU   OCP_LIPM_OPTI_PESSI_NSBU
#define NSH0   OCP_LIPM_OPTI_PESSI_NSH0
#define NSH    OCP_LIPM_OPTI_PESSI_NSH
#define NSHN   OCP_LIPM_OPTI_PESSI_NSHN
#define NSG    OCP_LIPM_OPTI_PESSI_NSG
#define NSPHI0 OCP_LIPM_OPTI_PESSI_NSPHI0
#define NSPHI  OCP_LIPM_OPTI_PESSI_NSPHI
#define NSPHIN OCP_LIPM_OPTI_PESSI_NSPHIN
#define NSGN   OCP_LIPM_OPTI_PESSI_NSGN
#define NSBXN  OCP_LIPM_OPTI_PESSI_NSBXN





// ** solver data **

ocp_lipm_opti_pessi_solver_capsule * ocp_lipm_opti_pessi_acados_create_capsule(void)
{
    void* capsule_mem = malloc(sizeof(ocp_lipm_opti_pessi_solver_capsule));
    ocp_lipm_opti_pessi_solver_capsule *capsule = (ocp_lipm_opti_pessi_solver_capsule *) capsule_mem;

    return capsule;
}


int ocp_lipm_opti_pessi_acados_free_capsule(ocp_lipm_opti_pessi_solver_capsule *capsule)
{
    free(capsule);
    return 0;
}


int ocp_lipm_opti_pessi_acados_create(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    int N_shooting_intervals = OCP_LIPM_OPTI_PESSI_N;
    double* new_time_steps = NULL; // NULL -> don't alter the code generated time-steps
    return ocp_lipm_opti_pessi_acados_create_with_discretization(capsule, N_shooting_intervals, new_time_steps);
}


int ocp_lipm_opti_pessi_acados_update_time_steps(ocp_lipm_opti_pessi_solver_capsule* capsule, int N, double* new_time_steps)
{

    if (N != capsule->nlp_solver_plan->N) {
        fprintf(stderr, "ocp_lipm_opti_pessi_acados_update_time_steps: given number of time steps (= %d) " \
            "differs from the currently allocated number of " \
            "time steps (= %d)!\n" \
            "Please recreate with new discretization and provide a new vector of time_stamps!\n",
            N, capsule->nlp_solver_plan->N);
        return 1;
    }

    ocp_nlp_config * nlp_config = capsule->nlp_config;
    ocp_nlp_dims * nlp_dims = capsule->nlp_dims;
    ocp_nlp_in * nlp_in = capsule->nlp_in;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &new_time_steps[i]);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &new_time_steps[i]);
    }
    return 0;

}

/**
 * Internal function for ocp_lipm_opti_pessi_acados_create: step 1
 */
void ocp_lipm_opti_pessi_acados_create_set_plan(ocp_nlp_plan_t* nlp_solver_plan, const int N)
{
    assert(N == nlp_solver_plan->N);

    /************************************************
    *  plan
    ************************************************/

    nlp_solver_plan->nlp_solver = SQP;

    nlp_solver_plan->ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->relaxed_ocp_qp_solver_plan.qp_solver = PARTIAL_CONDENSING_HPIPM;
    nlp_solver_plan->nlp_cost[0] = NONLINEAR_LS;
    for (int i = 1; i < N; i++)
        nlp_solver_plan->nlp_cost[i] = NONLINEAR_LS;

    nlp_solver_plan->nlp_cost[N] = NONLINEAR_LS;

    for (int i = 0; i < N; i++)
    {
        nlp_solver_plan->nlp_dynamics[i] = DISCRETE_MODEL;
        // discrete dynamics does not need sim solver option, this field is ignored
        nlp_solver_plan->sim_solver_plan[i].sim_solver = INVALID_SIM_SOLVER;
    }

    nlp_solver_plan->nlp_constraints[0] = BGH;

    for (int i = 1; i < N; i++)
    {
        nlp_solver_plan->nlp_constraints[i] = BGH;
    }
    nlp_solver_plan->nlp_constraints[N] = BGH;

    nlp_solver_plan->regularization = NO_REGULARIZE;

    nlp_solver_plan->globalization = MERIT_BACKTRACKING;
}


static ocp_nlp_dims* ocp_lipm_opti_pessi_acados_create_setup_dimensions(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    ocp_nlp_plan_t* nlp_solver_plan = capsule->nlp_solver_plan;
    const int N = nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;

    /************************************************
    *  dimensions
    ************************************************/
    #define NINTNP1MEMS 18
    int* intNp1mem = (int*)malloc( (N+1)*sizeof(int)*NINTNP1MEMS );

    int* nx    = intNp1mem + (N+1)*0;
    int* nu    = intNp1mem + (N+1)*1;
    int* nbx   = intNp1mem + (N+1)*2;
    int* nbu   = intNp1mem + (N+1)*3;
    int* nsbx  = intNp1mem + (N+1)*4;
    int* nsbu  = intNp1mem + (N+1)*5;
    int* nsg   = intNp1mem + (N+1)*6;
    int* nsh   = intNp1mem + (N+1)*7;
    int* nsphi = intNp1mem + (N+1)*8;
    int* ns    = intNp1mem + (N+1)*9;
    int* ng    = intNp1mem + (N+1)*10;
    int* nh    = intNp1mem + (N+1)*11;
    int* nphi  = intNp1mem + (N+1)*12;
    int* nz    = intNp1mem + (N+1)*13;
    int* ny    = intNp1mem + (N+1)*14;
    int* nr    = intNp1mem + (N+1)*15;
    int* nbxe  = intNp1mem + (N+1)*16;
    int* np  = intNp1mem + (N+1)*17;

    for (int i = 0; i < N+1; i++)
    {
        // common
        nx[i]     = NX;
        nu[i]     = NU;
        nz[i]     = NZ;
        ns[i]     = NS;
        // cost
        ny[i]     = NY;
        // constraints
        nbx[i]    = NBX;
        nbu[i]    = NBU;
        nsbx[i]   = NSBX;
        nsbu[i]   = NSBU;
        nsg[i]    = NSG;
        nsh[i]    = NSH;
        nsphi[i]  = NSPHI;
        ng[i]     = NG;
        nh[i]     = NH;
        nphi[i]   = NPHI;
        nr[i]     = NR;
        nbxe[i]   = 0;
        np[i]     = NP;
    }

    // for initial state
    nbx[0] = NBX0;
    nsbx[0] = 0;
    ns[0] = NS0;
    
    nbxe[0] = 22;
    
    ny[0] = NY0;
    nh[0] = NH0;
    nsh[0] = NSH0;
    nsphi[0] = NSPHI0;
    nphi[0] = NPHI0;


    // terminal - common
    nu[N]   = 0;
    nz[N]   = 0;
    ns[N]   = NSN;
    // cost
    ny[N]   = NYN;
    // constraint
    nbx[N]   = NBXN;
    nbu[N]   = 0;
    ng[N]    = NGN;
    nh[N]    = NHN;
    nphi[N]  = NPHIN;
    nr[N]    = 0;

    nsbx[N]  = NSBXN;
    nsbu[N]  = 0;
    nsg[N]   = NSGN;
    nsh[N]   = NSHN;
    nsphi[N] = NSPHIN;

    /* create and set ocp_nlp_dims */
    ocp_nlp_dims * nlp_dims = ocp_nlp_dims_create(nlp_config);

    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nx", nx);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nu", nu);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "nz", nz);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "ns", ns);
    ocp_nlp_dims_set_opt_vars(nlp_config, nlp_dims, "np", np);

    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "np_global", 0);
    ocp_nlp_dims_set_global(nlp_config, nlp_dims, "n_global_data", 0);

    for (int i = 0; i <= N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbx", &nbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbu", &nbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbx", &nsbx[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsbu", &nsbu[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "ng", &ng[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsg", &nsg[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nbxe", &nbxe[i]);
    }
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, 0, "ny", &ny[0]);
    for (int i = 1; i < N; i++)
        ocp_nlp_dims_set_cost(nlp_config, nlp_dims, i, "ny", &ny[i]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nh", &nh[0]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, 0, "nsh", &nsh[0]);

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nh", &nh[i]);
        ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, i, "nsh", &nsh[i]);
    }
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nh", &nh[N]);
    ocp_nlp_dims_set_constraints(nlp_config, nlp_dims, N, "nsh", &nsh[N]);
    ocp_nlp_dims_set_cost(nlp_config, nlp_dims, N, "ny", &ny[N]);
    free(intNp1mem);

    return nlp_dims;
}


/**
 * Internal function for ocp_lipm_opti_pessi_acados_create: step 3
 */
void ocp_lipm_opti_pessi_acados_create_setup_functions(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;

    /************************************************
    *  external functions
    ************************************************/

#define MAP_CASADI_FNC(__CAPSULE_FNC__, __MODEL_BASE_FNC__) do{ \
        capsule->__CAPSULE_FNC__.casadi_fun = & __MODEL_BASE_FNC__ ;\
        capsule->__CAPSULE_FNC__.casadi_n_in = & __MODEL_BASE_FNC__ ## _n_in; \
        capsule->__CAPSULE_FNC__.casadi_n_out = & __MODEL_BASE_FNC__ ## _n_out; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_in = & __MODEL_BASE_FNC__ ## _sparsity_in; \
        capsule->__CAPSULE_FNC__.casadi_sparsity_out = & __MODEL_BASE_FNC__ ## _sparsity_out; \
        capsule->__CAPSULE_FNC__.casadi_work = & __MODEL_BASE_FNC__ ## _work; \
        external_function_external_param_casadi_create(&capsule->__CAPSULE_FNC__, &ext_fun_opts); \
    } while(false)

    external_function_opts ext_fun_opts;
    external_function_opts_set_to_default(&ext_fun_opts);


    ext_fun_opts.external_workspace = true;
    if (N > 0)
    {
        MAP_CASADI_FNC(nl_constr_h_0_fun_jac, lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt);
        MAP_CASADI_FNC(nl_constr_h_0_fun, lipm_opti_pessi_constr_h_0_fun);
        // constraints.constr_type == "BGH" and dims.nh > 0
        capsule->nl_constr_h_fun_jac = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun_jac[i], lipm_opti_pessi_constr_h_fun_jac_uxt_zt);
        }
        capsule->nl_constr_h_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++) {
            MAP_CASADI_FNC(nl_constr_h_fun[i], lipm_opti_pessi_constr_h_fun);
        }
    
        // nonlinear least squares function
        MAP_CASADI_FNC(cost_y_0_fun, lipm_opti_pessi_cost_y_0_fun);
        MAP_CASADI_FNC(cost_y_0_fun_jac_ut_xt, lipm_opti_pessi_cost_y_0_fun_jac_ut_xt);



    
        // discrete dynamics
        capsule->discr_dyn_phi_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun[i], lipm_opti_pessi_dyn_disc_phi_fun);
        }

        capsule->discr_dyn_phi_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*N);
        for (int i = 0; i < N; i++)
        {
            MAP_CASADI_FNC(discr_dyn_phi_fun_jac_ut_xt[i], lipm_opti_pessi_dyn_disc_phi_fun_jac);
        }

    

    

    
        // nonlinear least squares cost
        capsule->cost_y_fun = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun[i], lipm_opti_pessi_cost_y_fun);
        }

        capsule->cost_y_fun_jac_ut_xt = (external_function_external_param_casadi *) malloc(sizeof(external_function_external_param_casadi)*(N-1));
        for (int i = 0; i < N-1; i++)
        {
            MAP_CASADI_FNC(cost_y_fun_jac_ut_xt[i], lipm_opti_pessi_cost_y_fun_jac_ut_xt);
        }
    } // N > 0
    // nonlinear least square function
    MAP_CASADI_FNC(cost_y_e_fun, lipm_opti_pessi_cost_y_e_fun);
    MAP_CASADI_FNC(cost_y_e_fun_jac_ut_xt, lipm_opti_pessi_cost_y_e_fun_jac_ut_xt);

#undef MAP_CASADI_FNC
}


/**
 * Internal function for ocp_lipm_opti_pessi_acados_create: step 5
 */
void ocp_lipm_opti_pessi_acados_create_set_default_parameters(ocp_lipm_opti_pessi_solver_capsule* capsule)
{

    const int N = capsule->nlp_solver_plan->N;

    // initialize parameters to initial value
    
    double* p = calloc(NP, sizeof(double));
    p[44] = 10;
    p[45] = -2;

    for (int i = 0; i <= N; i++) {
        ocp_lipm_opti_pessi_acados_update_params(capsule, i, p, NP);
    }
    free(p);


    // no global parameters defined
}


/**
 * Internal function for ocp_lipm_opti_pessi_acados_create: step 5
 */
void ocp_lipm_opti_pessi_acados_create_setup_nlp_in_numerical_values(ocp_lipm_opti_pessi_solver_capsule* capsule, const int N, double* new_time_steps)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    int tmp_int = 0;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;

    // set up time_steps and cost_scaling

    if (new_time_steps)
    {
        // NOTE: this sets scaling and time_steps
        ocp_lipm_opti_pessi_acados_update_time_steps(capsule, N, new_time_steps);
    }
    else
    {
        // set time_steps
    
        double time_step = 0.05000000000000001;
        for (int i = 0; i < N; i++)
        {
            ocp_nlp_in_set(nlp_config, nlp_dims, nlp_in, i, "Ts", &time_step);
        }
        // set cost scaling
        double* cost_scaling = malloc((N+1)*sizeof(double));
        cost_scaling[0] = 0.05000000000000001;
        cost_scaling[1] = 0.05000000000000001;
        cost_scaling[2] = 0.05000000000000001;
        cost_scaling[3] = 0.05000000000000001;
        cost_scaling[4] = 0.05000000000000001;
        cost_scaling[5] = 0.05000000000000001;
        cost_scaling[6] = 1;
        for (int i = 0; i <= N; i++)
        {
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "scaling", &cost_scaling[i]);
        }
        free(cost_scaling);
    }



    /**** Cost ****/
    double* yref_0 = calloc(NY0, sizeof(double));
    // change only the non-zero elements:
    yref_0[0] = 10;
    yref_0[1] = -2;
    yref_0[19] = 10;
    yref_0[20] = -2;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "yref", yref_0);
    free(yref_0);

   double* W_0 = calloc(NY0*NY0, sizeof(double));
    // change only the non-zero elements:
    W_0[0+(NY0) * 0] = 10;
    W_0[1+(NY0) * 1] = 10;
    W_0[2+(NY0) * 2] = 10;
    W_0[3+(NY0) * 3] = 5;
    W_0[4+(NY0) * 4] = 5;
    W_0[5+(NY0) * 5] = 1;
    W_0[6+(NY0) * 6] = 0.000001;
    W_0[7+(NY0) * 7] = 10;
    W_0[8+(NY0) * 8] = 10;
    W_0[9+(NY0) * 9] = 10;
    W_0[10+(NY0) * 10] = 10;
    W_0[11+(NY0) * 11] = 10000;
    W_0[12+(NY0) * 12] = 10000;
    W_0[13+(NY0) * 13] = 10000;
    W_0[14+(NY0) * 14] = 10000;
    W_0[15+(NY0) * 15] = 1;
    W_0[16+(NY0) * 16] = 0.001;
    W_0[17+(NY0) * 17] = 20;
    W_0[18+(NY0) * 18] = 20;
    W_0[19+(NY0) * 19] = 0.01;
    W_0[20+(NY0) * 20] = 0.01;
    W_0[21+(NY0) * 21] = 0.01;
    W_0[22+(NY0) * 22] = 0.005;
    W_0[23+(NY0) * 23] = 0.005;
    W_0[24+(NY0) * 24] = 0.001;
    W_0[25+(NY0) * 25] = 0.000000001;
    W_0[26+(NY0) * 26] = 0.01;
    W_0[27+(NY0) * 27] = 0.01;
    W_0[28+(NY0) * 28] = 0.01;
    W_0[29+(NY0) * 29] = 0.01;
    W_0[30+(NY0) * 30] = 10;
    W_0[31+(NY0) * 31] = 10;
    W_0[32+(NY0) * 32] = 10;
    W_0[33+(NY0) * 33] = 10;
    W_0[34+(NY0) * 34] = 0.001;
    W_0[35+(NY0) * 35] = 0.000001;
    W_0[36+(NY0) * 36] = 0.02;
    W_0[37+(NY0) * 37] = 0.02;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "W", W_0);
    free(W_0);
    double* yref = calloc(NY, sizeof(double));
    // change only the non-zero elements:
    yref[0] = 10;
    yref[1] = -2;
    yref[19] = 10;
    yref[20] = -2;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "yref", yref);
    }
    free(yref);
    double* W = calloc(NY*NY, sizeof(double));
    // change only the non-zero elements:
    W[0+(NY) * 0] = 10;
    W[1+(NY) * 1] = 10;
    W[2+(NY) * 2] = 10;
    W[3+(NY) * 3] = 5;
    W[4+(NY) * 4] = 5;
    W[5+(NY) * 5] = 1;
    W[6+(NY) * 6] = 0.000001;
    W[7+(NY) * 7] = 10;
    W[8+(NY) * 8] = 10;
    W[9+(NY) * 9] = 10;
    W[10+(NY) * 10] = 10;
    W[11+(NY) * 11] = 10000;
    W[12+(NY) * 12] = 10000;
    W[13+(NY) * 13] = 10000;
    W[14+(NY) * 14] = 10000;
    W[15+(NY) * 15] = 1;
    W[16+(NY) * 16] = 0.001;
    W[17+(NY) * 17] = 20;
    W[18+(NY) * 18] = 20;
    W[19+(NY) * 19] = 0.01;
    W[20+(NY) * 20] = 0.01;
    W[21+(NY) * 21] = 0.01;
    W[22+(NY) * 22] = 0.005;
    W[23+(NY) * 23] = 0.005;
    W[24+(NY) * 24] = 0.001;
    W[25+(NY) * 25] = 0.000000001;
    W[26+(NY) * 26] = 0.01;
    W[27+(NY) * 27] = 0.01;
    W[28+(NY) * 28] = 0.01;
    W[29+(NY) * 29] = 0.01;
    W[30+(NY) * 30] = 10;
    W[31+(NY) * 31] = 10;
    W[32+(NY) * 32] = 10;
    W[33+(NY) * 33] = 10;
    W[34+(NY) * 34] = 0.001;
    W[35+(NY) * 35] = 0.000001;
    W[36+(NY) * 36] = 0.02;
    W[37+(NY) * 37] = 0.02;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "W", W);
    }
    free(W);
    double* yref_e = calloc(NYN, sizeof(double));
    // change only the non-zero elements:
    yref_e[0] = 10;
    yref_e[1] = -2;
    yref_e[8] = 10;
    yref_e[9] = -2;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", yref_e);
    free(yref_e);

    double* W_e = calloc(NYN*NYN, sizeof(double));
    // change only the non-zero elements:
    W_e[0+(NYN) * 0] = 10;
    W_e[1+(NYN) * 1] = 10;
    W_e[2+(NYN) * 2] = 5;
    W_e[3+(NYN) * 3] = 5;
    W_e[4+(NYN) * 4] = 1;
    W_e[5+(NYN) * 5] = 0.000001;
    W_e[6+(NYN) * 6] = 20;
    W_e[7+(NYN) * 7] = 20;
    W_e[8+(NYN) * 8] = 0.01;
    W_e[9+(NYN) * 9] = 0.01;
    W_e[10+(NYN) * 10] = 0.005;
    W_e[11+(NYN) * 11] = 0.005;
    W_e[12+(NYN) * 12] = 0.001;
    W_e[13+(NYN) * 13] = 0.000000001;
    W_e[14+(NYN) * 14] = 0.02;
    W_e[15+(NYN) * 15] = 0.02;
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "W", W_e);
    free(W_e);



    // slacks initial
    double* zlu0_mem = calloc(4*NS0, sizeof(double));
    double* Zl_0 = zlu0_mem+NS0*0;
    double* Zu_0 = zlu0_mem+NS0*1;
    double* zl_0 = zlu0_mem+NS0*2;
    double* zu_0 = zlu0_mem+NS0*3;

    // change only the non-zero elements:
    Zl_0[64] = 1000;
    Zl_0[65] = 1000;
    Zl_0[130] = 1000;
    Zl_0[131] = 1000;
    Zu_0[0] = 10000;
    Zu_0[1] = 10000;
    Zu_0[2] = 10000;
    Zu_0[3] = 10000;
    Zu_0[4] = 10000;
    Zu_0[5] = 10000;
    Zu_0[6] = 10000;
    Zu_0[7] = 10000;
    Zu_0[8] = 10000;
    Zu_0[9] = 10000;
    Zu_0[10] = 10000;
    Zu_0[11] = 10000;
    Zu_0[12] = 10000;
    Zu_0[13] = 10000;
    Zu_0[14] = 10000;
    Zu_0[15] = 10000;
    Zu_0[16] = 10000;
    Zu_0[17] = 10000;
    Zu_0[18] = 10000;
    Zu_0[19] = 10000;
    Zu_0[20] = 10000;
    Zu_0[21] = 10000;
    Zu_0[22] = 10000;
    Zu_0[23] = 10000;
    Zu_0[24] = 10000;
    Zu_0[25] = 10000;
    Zu_0[26] = 10000;
    Zu_0[27] = 10000;
    Zu_0[28] = 10000;
    Zu_0[29] = 10000;
    Zu_0[30] = 10000;
    Zu_0[31] = 10000;
    Zu_0[32] = 10000;
    Zu_0[33] = 10000;
    Zu_0[34] = 10000;
    Zu_0[35] = 10000;
    Zu_0[36] = 10000;
    Zu_0[37] = 10000;
    Zu_0[38] = 10000;
    Zu_0[39] = 10000;
    Zu_0[40] = 10000;
    Zu_0[41] = 10000;
    Zu_0[42] = 10000;
    Zu_0[43] = 10000;
    Zu_0[44] = 10000;
    Zu_0[45] = 10000;
    Zu_0[46] = 10000;
    Zu_0[47] = 10000;
    Zu_0[48] = 10000;
    Zu_0[49] = 10000;
    Zu_0[50] = 10000;
    Zu_0[51] = 10000;
    Zu_0[52] = 10000;
    Zu_0[53] = 10000;
    Zu_0[54] = 10000;
    Zu_0[55] = 10000;
    Zu_0[56] = 10000;
    Zu_0[57] = 10000;
    Zu_0[58] = 10000;
    Zu_0[59] = 10000;
    Zu_0[60] = 10000;
    Zu_0[61] = 10000;
    Zu_0[62] = 10000;
    Zu_0[63] = 10000;
    Zu_0[64] = 1000;
    Zu_0[65] = 1000;
    Zu_0[66] = 10000;
    Zu_0[67] = 10000;
    Zu_0[68] = 10000;
    Zu_0[69] = 10000;
    Zu_0[70] = 10000;
    Zu_0[71] = 10000;
    Zu_0[72] = 10000;
    Zu_0[73] = 10000;
    Zu_0[74] = 10000;
    Zu_0[75] = 10000;
    Zu_0[76] = 10000;
    Zu_0[77] = 10000;
    Zu_0[78] = 10000;
    Zu_0[79] = 10000;
    Zu_0[80] = 10000;
    Zu_0[81] = 10000;
    Zu_0[82] = 10000;
    Zu_0[83] = 10000;
    Zu_0[84] = 10000;
    Zu_0[85] = 10000;
    Zu_0[86] = 10000;
    Zu_0[87] = 10000;
    Zu_0[88] = 10000;
    Zu_0[89] = 10000;
    Zu_0[90] = 10000;
    Zu_0[91] = 10000;
    Zu_0[92] = 10000;
    Zu_0[93] = 10000;
    Zu_0[94] = 10000;
    Zu_0[95] = 10000;
    Zu_0[96] = 10000;
    Zu_0[97] = 10000;
    Zu_0[98] = 10000;
    Zu_0[99] = 10000;
    Zu_0[100] = 10000;
    Zu_0[101] = 10000;
    Zu_0[102] = 10000;
    Zu_0[103] = 10000;
    Zu_0[104] = 10000;
    Zu_0[105] = 10000;
    Zu_0[106] = 10000;
    Zu_0[107] = 10000;
    Zu_0[108] = 10000;
    Zu_0[109] = 10000;
    Zu_0[110] = 10000;
    Zu_0[111] = 10000;
    Zu_0[112] = 10000;
    Zu_0[113] = 10000;
    Zu_0[114] = 10000;
    Zu_0[115] = 10000;
    Zu_0[116] = 10000;
    Zu_0[117] = 10000;
    Zu_0[118] = 10000;
    Zu_0[119] = 10000;
    Zu_0[120] = 10000;
    Zu_0[121] = 10000;
    Zu_0[122] = 10000;
    Zu_0[123] = 10000;
    Zu_0[124] = 10000;
    Zu_0[125] = 10000;
    Zu_0[126] = 10000;
    Zu_0[127] = 10000;
    Zu_0[128] = 10000;
    Zu_0[129] = 10000;
    Zu_0[130] = 1000;
    Zu_0[131] = 1000;
    zl_0[64] = 100;
    zl_0[65] = 100;
    zl_0[130] = 100;
    zl_0[131] = 100;
    zu_0[0] = 1000;
    zu_0[1] = 1000;
    zu_0[2] = 1000;
    zu_0[3] = 1000;
    zu_0[4] = 1000;
    zu_0[5] = 1000;
    zu_0[6] = 1000;
    zu_0[7] = 1000;
    zu_0[8] = 1000;
    zu_0[9] = 1000;
    zu_0[10] = 1000;
    zu_0[11] = 1000;
    zu_0[12] = 1000;
    zu_0[13] = 1000;
    zu_0[14] = 1000;
    zu_0[15] = 1000;
    zu_0[16] = 1000;
    zu_0[17] = 1000;
    zu_0[18] = 1000;
    zu_0[19] = 1000;
    zu_0[20] = 1000;
    zu_0[21] = 1000;
    zu_0[22] = 1000;
    zu_0[23] = 1000;
    zu_0[24] = 1000;
    zu_0[25] = 1000;
    zu_0[26] = 1000;
    zu_0[27] = 1000;
    zu_0[28] = 1000;
    zu_0[29] = 1000;
    zu_0[30] = 1000;
    zu_0[31] = 1000;
    zu_0[32] = 1000;
    zu_0[33] = 1000;
    zu_0[34] = 1000;
    zu_0[35] = 1000;
    zu_0[36] = 1000;
    zu_0[37] = 1000;
    zu_0[38] = 1000;
    zu_0[39] = 1000;
    zu_0[40] = 1000;
    zu_0[41] = 1000;
    zu_0[42] = 1000;
    zu_0[43] = 1000;
    zu_0[44] = 1000;
    zu_0[45] = 1000;
    zu_0[46] = 1000;
    zu_0[47] = 1000;
    zu_0[48] = 1000;
    zu_0[49] = 1000;
    zu_0[50] = 1000;
    zu_0[51] = 1000;
    zu_0[52] = 1000;
    zu_0[53] = 1000;
    zu_0[54] = 1000;
    zu_0[55] = 1000;
    zu_0[56] = 1000;
    zu_0[57] = 1000;
    zu_0[58] = 1000;
    zu_0[59] = 1000;
    zu_0[60] = 1000;
    zu_0[61] = 1000;
    zu_0[62] = 1000;
    zu_0[63] = 1000;
    zu_0[64] = 100;
    zu_0[65] = 100;
    zu_0[66] = 1000;
    zu_0[67] = 1000;
    zu_0[68] = 1000;
    zu_0[69] = 1000;
    zu_0[70] = 1000;
    zu_0[71] = 1000;
    zu_0[72] = 1000;
    zu_0[73] = 1000;
    zu_0[74] = 1000;
    zu_0[75] = 1000;
    zu_0[76] = 1000;
    zu_0[77] = 1000;
    zu_0[78] = 1000;
    zu_0[79] = 1000;
    zu_0[80] = 1000;
    zu_0[81] = 1000;
    zu_0[82] = 1000;
    zu_0[83] = 1000;
    zu_0[84] = 1000;
    zu_0[85] = 1000;
    zu_0[86] = 1000;
    zu_0[87] = 1000;
    zu_0[88] = 1000;
    zu_0[89] = 1000;
    zu_0[90] = 1000;
    zu_0[91] = 1000;
    zu_0[92] = 1000;
    zu_0[93] = 1000;
    zu_0[94] = 1000;
    zu_0[95] = 1000;
    zu_0[96] = 1000;
    zu_0[97] = 1000;
    zu_0[98] = 1000;
    zu_0[99] = 1000;
    zu_0[100] = 1000;
    zu_0[101] = 1000;
    zu_0[102] = 1000;
    zu_0[103] = 1000;
    zu_0[104] = 1000;
    zu_0[105] = 1000;
    zu_0[106] = 1000;
    zu_0[107] = 1000;
    zu_0[108] = 1000;
    zu_0[109] = 1000;
    zu_0[110] = 1000;
    zu_0[111] = 1000;
    zu_0[112] = 1000;
    zu_0[113] = 1000;
    zu_0[114] = 1000;
    zu_0[115] = 1000;
    zu_0[116] = 1000;
    zu_0[117] = 1000;
    zu_0[118] = 1000;
    zu_0[119] = 1000;
    zu_0[120] = 1000;
    zu_0[121] = 1000;
    zu_0[122] = 1000;
    zu_0[123] = 1000;
    zu_0[124] = 1000;
    zu_0[125] = 1000;
    zu_0[126] = 1000;
    zu_0[127] = 1000;
    zu_0[128] = 1000;
    zu_0[129] = 1000;
    zu_0[130] = 100;
    zu_0[131] = 100;

    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Zl", Zl_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "Zu", Zu_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "zl", zl_0);
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, 0, "zu", zu_0);
    free(zlu0_mem);
    // slacks
    double* zlumem = calloc(4*NS, sizeof(double));
    double* Zl = zlumem+NS*0;
    double* Zu = zlumem+NS*1;
    double* zl = zlumem+NS*2;
    double* zu = zlumem+NS*3;
    // change only the non-zero elements:
    Zl[0] = 1000;
    Zl[1] = 1000;
    Zl[66] = 1000;
    Zl[67] = 1000;
    Zl[132] = 1000;
    Zl[133] = 1000;
    Zu[0] = 1000;
    Zu[1] = 1000;
    Zu[2] = 10000;
    Zu[3] = 10000;
    Zu[4] = 10000;
    Zu[5] = 10000;
    Zu[6] = 10000;
    Zu[7] = 10000;
    Zu[8] = 10000;
    Zu[9] = 10000;
    Zu[10] = 10000;
    Zu[11] = 10000;
    Zu[12] = 10000;
    Zu[13] = 10000;
    Zu[14] = 10000;
    Zu[15] = 10000;
    Zu[16] = 10000;
    Zu[17] = 10000;
    Zu[18] = 10000;
    Zu[19] = 10000;
    Zu[20] = 10000;
    Zu[21] = 10000;
    Zu[22] = 10000;
    Zu[23] = 10000;
    Zu[24] = 10000;
    Zu[25] = 10000;
    Zu[26] = 10000;
    Zu[27] = 10000;
    Zu[28] = 10000;
    Zu[29] = 10000;
    Zu[30] = 10000;
    Zu[31] = 10000;
    Zu[32] = 10000;
    Zu[33] = 10000;
    Zu[34] = 10000;
    Zu[35] = 10000;
    Zu[36] = 10000;
    Zu[37] = 10000;
    Zu[38] = 10000;
    Zu[39] = 10000;
    Zu[40] = 10000;
    Zu[41] = 10000;
    Zu[42] = 10000;
    Zu[43] = 10000;
    Zu[44] = 10000;
    Zu[45] = 10000;
    Zu[46] = 10000;
    Zu[47] = 10000;
    Zu[48] = 10000;
    Zu[49] = 10000;
    Zu[50] = 10000;
    Zu[51] = 10000;
    Zu[52] = 10000;
    Zu[53] = 10000;
    Zu[54] = 10000;
    Zu[55] = 10000;
    Zu[56] = 10000;
    Zu[57] = 10000;
    Zu[58] = 10000;
    Zu[59] = 10000;
    Zu[60] = 10000;
    Zu[61] = 10000;
    Zu[62] = 10000;
    Zu[63] = 10000;
    Zu[64] = 10000;
    Zu[65] = 10000;
    Zu[66] = 1000;
    Zu[67] = 1000;
    Zu[68] = 10000;
    Zu[69] = 10000;
    Zu[70] = 10000;
    Zu[71] = 10000;
    Zu[72] = 10000;
    Zu[73] = 10000;
    Zu[74] = 10000;
    Zu[75] = 10000;
    Zu[76] = 10000;
    Zu[77] = 10000;
    Zu[78] = 10000;
    Zu[79] = 10000;
    Zu[80] = 10000;
    Zu[81] = 10000;
    Zu[82] = 10000;
    Zu[83] = 10000;
    Zu[84] = 10000;
    Zu[85] = 10000;
    Zu[86] = 10000;
    Zu[87] = 10000;
    Zu[88] = 10000;
    Zu[89] = 10000;
    Zu[90] = 10000;
    Zu[91] = 10000;
    Zu[92] = 10000;
    Zu[93] = 10000;
    Zu[94] = 10000;
    Zu[95] = 10000;
    Zu[96] = 10000;
    Zu[97] = 10000;
    Zu[98] = 10000;
    Zu[99] = 10000;
    Zu[100] = 10000;
    Zu[101] = 10000;
    Zu[102] = 10000;
    Zu[103] = 10000;
    Zu[104] = 10000;
    Zu[105] = 10000;
    Zu[106] = 10000;
    Zu[107] = 10000;
    Zu[108] = 10000;
    Zu[109] = 10000;
    Zu[110] = 10000;
    Zu[111] = 10000;
    Zu[112] = 10000;
    Zu[113] = 10000;
    Zu[114] = 10000;
    Zu[115] = 10000;
    Zu[116] = 10000;
    Zu[117] = 10000;
    Zu[118] = 10000;
    Zu[119] = 10000;
    Zu[120] = 10000;
    Zu[121] = 10000;
    Zu[122] = 10000;
    Zu[123] = 10000;
    Zu[124] = 10000;
    Zu[125] = 10000;
    Zu[126] = 10000;
    Zu[127] = 10000;
    Zu[128] = 10000;
    Zu[129] = 10000;
    Zu[130] = 10000;
    Zu[131] = 10000;
    Zu[132] = 1000;
    Zu[133] = 1000;
    zl[0] = 100;
    zl[1] = 100;
    zl[66] = 100;
    zl[67] = 100;
    zl[132] = 100;
    zl[133] = 100;
    zu[0] = 100;
    zu[1] = 100;
    zu[2] = 1000;
    zu[3] = 1000;
    zu[4] = 1000;
    zu[5] = 1000;
    zu[6] = 1000;
    zu[7] = 1000;
    zu[8] = 1000;
    zu[9] = 1000;
    zu[10] = 1000;
    zu[11] = 1000;
    zu[12] = 1000;
    zu[13] = 1000;
    zu[14] = 1000;
    zu[15] = 1000;
    zu[16] = 1000;
    zu[17] = 1000;
    zu[18] = 1000;
    zu[19] = 1000;
    zu[20] = 1000;
    zu[21] = 1000;
    zu[22] = 1000;
    zu[23] = 1000;
    zu[24] = 1000;
    zu[25] = 1000;
    zu[26] = 1000;
    zu[27] = 1000;
    zu[28] = 1000;
    zu[29] = 1000;
    zu[30] = 1000;
    zu[31] = 1000;
    zu[32] = 1000;
    zu[33] = 1000;
    zu[34] = 1000;
    zu[35] = 1000;
    zu[36] = 1000;
    zu[37] = 1000;
    zu[38] = 1000;
    zu[39] = 1000;
    zu[40] = 1000;
    zu[41] = 1000;
    zu[42] = 1000;
    zu[43] = 1000;
    zu[44] = 1000;
    zu[45] = 1000;
    zu[46] = 1000;
    zu[47] = 1000;
    zu[48] = 1000;
    zu[49] = 1000;
    zu[50] = 1000;
    zu[51] = 1000;
    zu[52] = 1000;
    zu[53] = 1000;
    zu[54] = 1000;
    zu[55] = 1000;
    zu[56] = 1000;
    zu[57] = 1000;
    zu[58] = 1000;
    zu[59] = 1000;
    zu[60] = 1000;
    zu[61] = 1000;
    zu[62] = 1000;
    zu[63] = 1000;
    zu[64] = 1000;
    zu[65] = 1000;
    zu[66] = 100;
    zu[67] = 100;
    zu[68] = 1000;
    zu[69] = 1000;
    zu[70] = 1000;
    zu[71] = 1000;
    zu[72] = 1000;
    zu[73] = 1000;
    zu[74] = 1000;
    zu[75] = 1000;
    zu[76] = 1000;
    zu[77] = 1000;
    zu[78] = 1000;
    zu[79] = 1000;
    zu[80] = 1000;
    zu[81] = 1000;
    zu[82] = 1000;
    zu[83] = 1000;
    zu[84] = 1000;
    zu[85] = 1000;
    zu[86] = 1000;
    zu[87] = 1000;
    zu[88] = 1000;
    zu[89] = 1000;
    zu[90] = 1000;
    zu[91] = 1000;
    zu[92] = 1000;
    zu[93] = 1000;
    zu[94] = 1000;
    zu[95] = 1000;
    zu[96] = 1000;
    zu[97] = 1000;
    zu[98] = 1000;
    zu[99] = 1000;
    zu[100] = 1000;
    zu[101] = 1000;
    zu[102] = 1000;
    zu[103] = 1000;
    zu[104] = 1000;
    zu[105] = 1000;
    zu[106] = 1000;
    zu[107] = 1000;
    zu[108] = 1000;
    zu[109] = 1000;
    zu[110] = 1000;
    zu[111] = 1000;
    zu[112] = 1000;
    zu[113] = 1000;
    zu[114] = 1000;
    zu[115] = 1000;
    zu[116] = 1000;
    zu[117] = 1000;
    zu[118] = 1000;
    zu[119] = 1000;
    zu[120] = 1000;
    zu[121] = 1000;
    zu[122] = 1000;
    zu[123] = 1000;
    zu[124] = 1000;
    zu[125] = 1000;
    zu[126] = 1000;
    zu[127] = 1000;
    zu[128] = 1000;
    zu[129] = 1000;
    zu[130] = 1000;
    zu[131] = 1000;
    zu[132] = 100;
    zu[133] = 100;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zl", Zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "Zu", Zu);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zl", zl);
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, i, "zu", zu);
    }
    free(zlumem);



    /**** Constraints ****/

    // bounds for initial stage
    // x0
    int* idxbx0 = malloc(NBX0 * sizeof(int));
    idxbx0[0] = 0;
    idxbx0[1] = 1;
    idxbx0[2] = 2;
    idxbx0[3] = 3;
    idxbx0[4] = 4;
    idxbx0[5] = 5;
    idxbx0[6] = 6;
    idxbx0[7] = 7;
    idxbx0[8] = 8;
    idxbx0[9] = 9;
    idxbx0[10] = 10;
    idxbx0[11] = 11;
    idxbx0[12] = 12;
    idxbx0[13] = 13;
    idxbx0[14] = 14;
    idxbx0[15] = 15;
    idxbx0[16] = 16;
    idxbx0[17] = 17;
    idxbx0[18] = 18;
    idxbx0[19] = 19;
    idxbx0[20] = 20;
    idxbx0[21] = 21;

    double* lubx0 = calloc(2*NBX0, sizeof(double));
    double* lbx0 = lubx0;
    double* ubx0 = lubx0 + NBX0;
    // change only the non-zero elements:
    lbx0[0] = -0.8;
    ubx0[0] = -0.8;
    lbx0[1] = -0.5;
    ubx0[1] = -0.5;
    lbx0[6] = -0.5593;
    ubx0[6] = -0.5593;
    lbx0[7] = -0.634;
    ubx0[7] = -0.634;
    lbx0[8] = -1.0407;
    ubx0[8] = -1.0407;
    lbx0[9] = -0.366;
    ubx0[9] = -0.366;
    lbx0[11] = -0.8;
    ubx0[11] = -0.8;
    lbx0[12] = -0.5;
    ubx0[12] = -0.5;
    lbx0[17] = -0.5593;
    ubx0[17] = -0.5593;
    lbx0[18] = -0.634;
    ubx0[18] = -0.634;
    lbx0[19] = -1.0407;
    ubx0[19] = -1.0407;
    lbx0[20] = -0.366;
    ubx0[20] = -0.366;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbx", idxbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lbx", lbx0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ubx", ubx0);
    free(idxbx0);
    free(lubx0);
    // idxbxe_0
    int* idxbxe_0 = malloc(22 * sizeof(int));
    idxbxe_0[0] = 0;
    idxbxe_0[1] = 1;
    idxbxe_0[2] = 2;
    idxbxe_0[3] = 3;
    idxbxe_0[4] = 4;
    idxbxe_0[5] = 5;
    idxbxe_0[6] = 6;
    idxbxe_0[7] = 7;
    idxbxe_0[8] = 8;
    idxbxe_0[9] = 9;
    idxbxe_0[10] = 10;
    idxbxe_0[11] = 11;
    idxbxe_0[12] = 12;
    idxbxe_0[13] = 13;
    idxbxe_0[14] = 14;
    idxbxe_0[15] = 15;
    idxbxe_0[16] = 16;
    idxbxe_0[17] = 17;
    idxbxe_0[18] = 18;
    idxbxe_0[19] = 19;
    idxbxe_0[20] = 20;
    idxbxe_0[21] = 21;
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxbxe", idxbxe_0);
    free(idxbxe_0);



    // set up nonlinear constraints for last stage
    double* luh_0 = calloc(2*NH0, sizeof(double));
    double* lh_0 = luh_0;
    double* uh_0 = luh_0 + NH0;
    lh_0[12] = -1000000;
    lh_0[13] = -1000000;
    lh_0[14] = 0.5;
    lh_0[15] = -1000000;
    lh_0[16] = -1000000;
    lh_0[17] = -1000000;
    lh_0[18] = -1000000;
    lh_0[19] = -1000000;
    lh_0[20] = -1000000;
    lh_0[22] = 0.5;
    lh_0[23] = -1000000;
    lh_0[24] = -1000000;
    lh_0[25] = -1000000;
    lh_0[26] = -1000000;
    lh_0[27] = -1000000;
    lh_0[28] = -1000000;
    lh_0[30] = 0.5;
    lh_0[31] = -1000000;
    lh_0[32] = -1000000;
    lh_0[33] = -1000000;
    lh_0[34] = -1000000;
    lh_0[35] = -1000000;
    lh_0[36] = -1000000;
    lh_0[38] = 0.5;
    lh_0[39] = -1000000;
    lh_0[40] = -1000000;
    lh_0[41] = -1000000;
    lh_0[42] = -1000000;
    lh_0[43] = -1000000;
    lh_0[44] = -1000000;
    lh_0[46] = 0.5;
    lh_0[47] = -1000000;
    lh_0[48] = -1000000;
    lh_0[49] = -1000000;
    lh_0[50] = -1000000;
    lh_0[51] = -1000000;
    lh_0[52] = -1000000;
    lh_0[54] = 0.5;
    lh_0[55] = -1000000;
    lh_0[56] = -1000000;
    lh_0[57] = -1000000;
    lh_0[58] = -1000000;
    lh_0[59] = -1000000;
    lh_0[60] = -1000000;
    lh_0[62] = 0.5;
    lh_0[63] = -1000000;
    lh_0[64] = -1000000;
    lh_0[65] = -1000000;
    lh_0[66] = -1000000;
    lh_0[67] = -1000000;
    lh_0[68] = -1000000;
    lh_0[70] = 0.5;
    lh_0[71] = -1000000;
    lh_0[72] = -1000000;
    lh_0[73] = -1000000;
    lh_0[74] = -1000000;
    lh_0[75] = -1000000;
    lh_0[76] = -1000000;
    lh_0[78] = 0.5;
    lh_0[79] = -1000000;
    lh_0[80] = -1000000;
    lh_0[81] = -1000000;
    lh_0[82] = -1000000;
    lh_0[83] = -1000000;
    lh_0[84] = -1000000;
    lh_0[86] = 0.5;
    lh_0[87] = -1000000;
    lh_0[88] = -1000000;
    lh_0[89] = -1000000;
    lh_0[90] = -1000000;
    lh_0[91] = -1000000;
    lh_0[92] = -1000000;
    lh_0[94] = -1.5;
    lh_0[95] = -0.45;
    lh_0[100] = -1000000;
    lh_0[101] = -1000000;
    lh_0[102] = 0.5;
    lh_0[103] = -1000000;
    lh_0[104] = -1000000;
    lh_0[105] = -1000000;
    lh_0[106] = -1000000;
    lh_0[107] = -1000000;
    lh_0[108] = -1000000;
    lh_0[110] = 0.5;
    lh_0[111] = -1000000;
    lh_0[112] = -1000000;
    lh_0[113] = -1000000;
    lh_0[114] = -1000000;
    lh_0[115] = -1000000;
    lh_0[116] = -1000000;
    lh_0[118] = 0.5;
    lh_0[119] = -1000000;
    lh_0[120] = -1000000;
    lh_0[121] = -1000000;
    lh_0[122] = -1000000;
    lh_0[123] = -1000000;
    lh_0[124] = -1000000;
    lh_0[126] = 0.5;
    lh_0[127] = -1000000;
    lh_0[128] = -1000000;
    lh_0[129] = -1000000;
    lh_0[130] = -1000000;
    lh_0[131] = -1000000;
    lh_0[132] = -1000000;
    lh_0[134] = 0.5;
    lh_0[135] = -1000000;
    lh_0[136] = -1000000;
    lh_0[137] = -1000000;
    lh_0[138] = -1000000;
    lh_0[139] = -1000000;
    lh_0[140] = -1000000;
    lh_0[142] = 0.5;
    lh_0[143] = -1000000;
    lh_0[144] = -1000000;
    lh_0[145] = -1000000;
    lh_0[146] = -1000000;
    lh_0[147] = -1000000;
    lh_0[148] = -1000000;
    lh_0[150] = 0.5;
    lh_0[151] = -1000000;
    lh_0[152] = -1000000;
    lh_0[153] = -1000000;
    lh_0[154] = -1000000;
    lh_0[155] = -1000000;
    lh_0[156] = -1000000;
    lh_0[158] = 0.5;
    lh_0[159] = -1000000;
    lh_0[160] = -1000000;
    lh_0[161] = -1000000;
    lh_0[162] = -1000000;
    lh_0[163] = -1000000;
    lh_0[164] = -1000000;
    lh_0[166] = 0.5;
    lh_0[167] = -1000000;
    lh_0[168] = -1000000;
    lh_0[169] = -1000000;
    lh_0[170] = -1000000;
    lh_0[171] = -1000000;
    lh_0[172] = -1000000;
    lh_0[174] = 0.5;
    lh_0[175] = -1000000;
    lh_0[176] = -1000000;
    lh_0[177] = -1000000;
    lh_0[178] = -1000000;
    lh_0[179] = -1000000;
    lh_0[180] = -1000000;
    lh_0[182] = -1.5;
    lh_0[183] = -0.45;
    uh_0[8] = 0.09;
    uh_0[9] = 0.09;
    uh_0[10] = 0.09;
    uh_0[11] = 0.09;
    uh_0[14] = 1;
    uh_0[21] = 1000000;
    uh_0[22] = 1;
    uh_0[29] = 1000000;
    uh_0[30] = 1;
    uh_0[37] = 1000000;
    uh_0[38] = 1;
    uh_0[45] = 1000000;
    uh_0[46] = 1;
    uh_0[53] = 1000000;
    uh_0[54] = 1;
    uh_0[61] = 1000000;
    uh_0[62] = 1;
    uh_0[69] = 1000000;
    uh_0[70] = 1;
    uh_0[77] = 1000000;
    uh_0[78] = 1;
    uh_0[85] = 1000000;
    uh_0[86] = 1;
    uh_0[93] = 1000000;
    uh_0[94] = 1.5;
    uh_0[95] = 0.45;
    uh_0[96] = 0.09;
    uh_0[97] = 0.09;
    uh_0[98] = 0.09;
    uh_0[99] = 0.09;
    uh_0[102] = 1;
    uh_0[109] = 1000000;
    uh_0[110] = 1;
    uh_0[117] = 1000000;
    uh_0[118] = 1;
    uh_0[125] = 1000000;
    uh_0[126] = 1;
    uh_0[133] = 1000000;
    uh_0[134] = 1;
    uh_0[141] = 1000000;
    uh_0[142] = 1;
    uh_0[149] = 1000000;
    uh_0[150] = 1;
    uh_0[157] = 1000000;
    uh_0[158] = 1;
    uh_0[165] = 1000000;
    uh_0[166] = 1;
    uh_0[173] = 1000000;
    uh_0[174] = 1;
    uh_0[181] = 1000000;
    uh_0[182] = 1.5;
    uh_0[183] = 0.45;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lh", lh_0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "uh", uh_0);
    free(luh_0);






    // set up soft bounds for nonlinear constraints
    int* idxsh_0 = malloc(NSH0 * sizeof(int));
    idxsh_0[0] = 8;
    idxsh_0[1] = 9;
    idxsh_0[2] = 10;
    idxsh_0[3] = 11;
    idxsh_0[4] = 15;
    idxsh_0[5] = 16;
    idxsh_0[6] = 17;
    idxsh_0[7] = 18;
    idxsh_0[8] = 19;
    idxsh_0[9] = 20;
    idxsh_0[10] = 23;
    idxsh_0[11] = 24;
    idxsh_0[12] = 25;
    idxsh_0[13] = 26;
    idxsh_0[14] = 27;
    idxsh_0[15] = 28;
    idxsh_0[16] = 31;
    idxsh_0[17] = 32;
    idxsh_0[18] = 33;
    idxsh_0[19] = 34;
    idxsh_0[20] = 35;
    idxsh_0[21] = 36;
    idxsh_0[22] = 39;
    idxsh_0[23] = 40;
    idxsh_0[24] = 41;
    idxsh_0[25] = 42;
    idxsh_0[26] = 43;
    idxsh_0[27] = 44;
    idxsh_0[28] = 47;
    idxsh_0[29] = 48;
    idxsh_0[30] = 49;
    idxsh_0[31] = 50;
    idxsh_0[32] = 51;
    idxsh_0[33] = 52;
    idxsh_0[34] = 55;
    idxsh_0[35] = 56;
    idxsh_0[36] = 57;
    idxsh_0[37] = 58;
    idxsh_0[38] = 59;
    idxsh_0[39] = 60;
    idxsh_0[40] = 63;
    idxsh_0[41] = 64;
    idxsh_0[42] = 65;
    idxsh_0[43] = 66;
    idxsh_0[44] = 67;
    idxsh_0[45] = 68;
    idxsh_0[46] = 71;
    idxsh_0[47] = 72;
    idxsh_0[48] = 73;
    idxsh_0[49] = 74;
    idxsh_0[50] = 75;
    idxsh_0[51] = 76;
    idxsh_0[52] = 79;
    idxsh_0[53] = 80;
    idxsh_0[54] = 81;
    idxsh_0[55] = 82;
    idxsh_0[56] = 83;
    idxsh_0[57] = 84;
    idxsh_0[58] = 87;
    idxsh_0[59] = 88;
    idxsh_0[60] = 89;
    idxsh_0[61] = 90;
    idxsh_0[62] = 91;
    idxsh_0[63] = 92;
    idxsh_0[64] = 94;
    idxsh_0[65] = 95;
    idxsh_0[66] = 96;
    idxsh_0[67] = 97;
    idxsh_0[68] = 98;
    idxsh_0[69] = 99;
    idxsh_0[70] = 103;
    idxsh_0[71] = 104;
    idxsh_0[72] = 105;
    idxsh_0[73] = 106;
    idxsh_0[74] = 107;
    idxsh_0[75] = 108;
    idxsh_0[76] = 111;
    idxsh_0[77] = 112;
    idxsh_0[78] = 113;
    idxsh_0[79] = 114;
    idxsh_0[80] = 115;
    idxsh_0[81] = 116;
    idxsh_0[82] = 119;
    idxsh_0[83] = 120;
    idxsh_0[84] = 121;
    idxsh_0[85] = 122;
    idxsh_0[86] = 123;
    idxsh_0[87] = 124;
    idxsh_0[88] = 127;
    idxsh_0[89] = 128;
    idxsh_0[90] = 129;
    idxsh_0[91] = 130;
    idxsh_0[92] = 131;
    idxsh_0[93] = 132;
    idxsh_0[94] = 135;
    idxsh_0[95] = 136;
    idxsh_0[96] = 137;
    idxsh_0[97] = 138;
    idxsh_0[98] = 139;
    idxsh_0[99] = 140;
    idxsh_0[100] = 143;
    idxsh_0[101] = 144;
    idxsh_0[102] = 145;
    idxsh_0[103] = 146;
    idxsh_0[104] = 147;
    idxsh_0[105] = 148;
    idxsh_0[106] = 151;
    idxsh_0[107] = 152;
    idxsh_0[108] = 153;
    idxsh_0[109] = 154;
    idxsh_0[110] = 155;
    idxsh_0[111] = 156;
    idxsh_0[112] = 159;
    idxsh_0[113] = 160;
    idxsh_0[114] = 161;
    idxsh_0[115] = 162;
    idxsh_0[116] = 163;
    idxsh_0[117] = 164;
    idxsh_0[118] = 167;
    idxsh_0[119] = 168;
    idxsh_0[120] = 169;
    idxsh_0[121] = 170;
    idxsh_0[122] = 171;
    idxsh_0[123] = 172;
    idxsh_0[124] = 175;
    idxsh_0[125] = 176;
    idxsh_0[126] = 177;
    idxsh_0[127] = 178;
    idxsh_0[128] = 179;
    idxsh_0[129] = 180;
    idxsh_0[130] = 182;
    idxsh_0[131] = 183;
    double* lush_0 = calloc(2*NSH0, sizeof(double));
    double* lsh_0 = lush_0;
    double* ush_0 = lush_0 + NSH0;

    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxsh", idxsh_0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lsh", lsh_0);
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "ush", ush_0);
    free(idxsh_0);
    free(lush_0);



    /* constraints that are the same for initial and intermediate */
    // u
    int* idxbu = malloc(NBU * sizeof(int));
    idxbu[0] = 4;
    idxbu[1] = 5;
    idxbu[2] = 6;
    idxbu[3] = 7;
    idxbu[4] = 8;
    idxbu[5] = 9;
    idxbu[6] = 10;
    idxbu[7] = 11;
    idxbu[8] = 12;
    idxbu[9] = 13;
    idxbu[10] = 14;
    idxbu[11] = 15;
    idxbu[12] = 16;
    idxbu[13] = 17;
    idxbu[14] = 18;
    idxbu[15] = 19;
    idxbu[16] = 20;
    idxbu[17] = 21;
    idxbu[18] = 22;
    idxbu[19] = 23;
    idxbu[20] = 24;
    idxbu[21] = 25;
    idxbu[22] = 26;
    idxbu[23] = 27;
    idxbu[24] = 28;
    idxbu[25] = 29;
    idxbu[26] = 30;
    idxbu[27] = 31;
    idxbu[28] = 32;
    idxbu[29] = 33;
    idxbu[30] = 34;
    idxbu[31] = 35;
    idxbu[32] = 36;
    idxbu[33] = 37;
    idxbu[34] = 42;
    idxbu[35] = 43;
    idxbu[36] = 44;
    idxbu[37] = 45;
    idxbu[38] = 46;
    idxbu[39] = 47;
    idxbu[40] = 48;
    idxbu[41] = 49;
    idxbu[42] = 50;
    idxbu[43] = 51;
    idxbu[44] = 52;
    idxbu[45] = 53;
    idxbu[46] = 54;
    idxbu[47] = 55;
    idxbu[48] = 56;
    idxbu[49] = 57;
    idxbu[50] = 58;
    idxbu[51] = 59;
    idxbu[52] = 60;
    idxbu[53] = 61;
    idxbu[54] = 62;
    idxbu[55] = 63;
    idxbu[56] = 64;
    idxbu[57] = 65;
    idxbu[58] = 66;
    idxbu[59] = 67;
    idxbu[60] = 68;
    idxbu[61] = 69;
    idxbu[62] = 70;
    idxbu[63] = 71;
    idxbu[64] = 72;
    idxbu[65] = 73;
    idxbu[66] = 74;
    idxbu[67] = 75;
    double* lubu = calloc(2*NBU, sizeof(double));
    double* lbu = lubu;
    double* ubu = lubu + NBU;
    lbu[0] = 0.1;
    ubu[0] = 0.9;
    ubu[1] = 1;
    ubu[2] = 1;
    lbu[3] = 0.2;
    ubu[3] = 0.3;
    lbu[4] = -1;
    ubu[4] = 1;
    lbu[5] = -1;
    ubu[5] = 1;
    lbu[6] = -100;
    ubu[6] = 100;
    lbu[7] = -1;
    ubu[7] = 1;
    lbu[8] = -1;
    ubu[8] = 1;
    lbu[9] = -100;
    ubu[9] = 100;
    lbu[10] = -1;
    ubu[10] = 1;
    lbu[11] = -1;
    ubu[11] = 1;
    lbu[12] = -100;
    ubu[12] = 100;
    lbu[13] = -1;
    ubu[13] = 1;
    lbu[14] = -1;
    ubu[14] = 1;
    lbu[15] = -100;
    ubu[15] = 100;
    lbu[16] = -1;
    ubu[16] = 1;
    lbu[17] = -1;
    ubu[17] = 1;
    lbu[18] = -100;
    ubu[18] = 100;
    lbu[19] = -1;
    ubu[19] = 1;
    lbu[20] = -1;
    ubu[20] = 1;
    lbu[21] = -100;
    ubu[21] = 100;
    lbu[22] = -1;
    ubu[22] = 1;
    lbu[23] = -1;
    ubu[23] = 1;
    lbu[24] = -100;
    ubu[24] = 100;
    lbu[25] = -1;
    ubu[25] = 1;
    lbu[26] = -1;
    ubu[26] = 1;
    lbu[27] = -100;
    ubu[27] = 100;
    lbu[28] = -1;
    ubu[28] = 1;
    lbu[29] = -1;
    ubu[29] = 1;
    lbu[30] = -100;
    ubu[30] = 100;
    lbu[31] = -1;
    ubu[31] = 1;
    lbu[32] = -1;
    ubu[32] = 1;
    lbu[33] = -100;
    ubu[33] = 100;
    lbu[34] = 0.1;
    ubu[34] = 0.9;
    ubu[35] = 1;
    ubu[36] = 1;
    lbu[37] = 0.2;
    ubu[37] = 0.3;
    lbu[38] = -1;
    ubu[38] = 1;
    lbu[39] = -1;
    ubu[39] = 1;
    lbu[40] = -100;
    ubu[40] = 100;
    lbu[41] = -1;
    ubu[41] = 1;
    lbu[42] = -1;
    ubu[42] = 1;
    lbu[43] = -100;
    ubu[43] = 100;
    lbu[44] = -1;
    ubu[44] = 1;
    lbu[45] = -1;
    ubu[45] = 1;
    lbu[46] = -100;
    ubu[46] = 100;
    lbu[47] = -1;
    ubu[47] = 1;
    lbu[48] = -1;
    ubu[48] = 1;
    lbu[49] = -100;
    ubu[49] = 100;
    lbu[50] = -1;
    ubu[50] = 1;
    lbu[51] = -1;
    ubu[51] = 1;
    lbu[52] = -100;
    ubu[52] = 100;
    lbu[53] = -1;
    ubu[53] = 1;
    lbu[54] = -1;
    ubu[54] = 1;
    lbu[55] = -100;
    ubu[55] = 100;
    lbu[56] = -1;
    ubu[56] = 1;
    lbu[57] = -1;
    ubu[57] = 1;
    lbu[58] = -100;
    ubu[58] = 100;
    lbu[59] = -1;
    ubu[59] = 1;
    lbu[60] = -1;
    ubu[60] = 1;
    lbu[61] = -100;
    ubu[61] = 100;
    lbu[62] = -1;
    ubu[62] = 1;
    lbu[63] = -1;
    ubu[63] = 1;
    lbu[64] = -100;
    ubu[64] = 100;
    lbu[65] = -1;
    ubu[65] = 1;
    lbu[66] = -1;
    ubu[66] = 1;
    lbu[67] = -100;
    ubu[67] = 100;

    for (int i = 0; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbu", idxbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbu", lbu);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubu", ubu);
    }
    free(idxbu);
    free(lubu);






    /* Path constraints */

    // x
    int* idxbx = malloc(NBX * sizeof(int));
    idxbx[0] = 5;
    idxbx[1] = 10;
    idxbx[2] = 16;
    idxbx[3] = 21;
    double* lubx = calloc(2*NBX, sizeof(double));
    double* lbx = lubx;
    double* ubx = lubx + NBX;
    lbx[0] = -0.6;
    ubx[0] = 0.6;
    ubx[1] = 100;
    lbx[2] = -0.6;
    ubx[2] = 0.6;
    ubx[3] = 100;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxbx", idxbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lbx", lbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ubx", ubx);
    }
    free(idxbx);
    free(lubx);


    // set up nonlinear constraints for stage 1 to N-1
    double* luh = calloc(2*NH, sizeof(double));
    double* lh = luh;
    double* uh = luh + NH;
    lh[4] = -1000000;
    lh[5] = -1000000;
    lh[6] = 0.5;
    lh[7] = -1000000;
    lh[8] = -1000000;
    lh[9] = -1000000;
    lh[10] = -1000000;
    lh[11] = -1000000;
    lh[12] = -1000000;
    lh[14] = 0.5;
    lh[15] = -1000000;
    lh[16] = -1000000;
    lh[17] = -1000000;
    lh[18] = -1000000;
    lh[19] = -1000000;
    lh[20] = -1000000;
    lh[22] = 0.5;
    lh[23] = -1000000;
    lh[24] = -1000000;
    lh[25] = -1000000;
    lh[26] = -1000000;
    lh[27] = -1000000;
    lh[28] = -1000000;
    lh[30] = 0.5;
    lh[31] = -1000000;
    lh[32] = -1000000;
    lh[33] = -1000000;
    lh[34] = -1000000;
    lh[35] = -1000000;
    lh[36] = -1000000;
    lh[38] = 0.5;
    lh[39] = -1000000;
    lh[40] = -1000000;
    lh[41] = -1000000;
    lh[42] = -1000000;
    lh[43] = -1000000;
    lh[44] = -1000000;
    lh[46] = 0.5;
    lh[47] = -1000000;
    lh[48] = -1000000;
    lh[49] = -1000000;
    lh[50] = -1000000;
    lh[51] = -1000000;
    lh[52] = -1000000;
    lh[54] = 0.5;
    lh[55] = -1000000;
    lh[56] = -1000000;
    lh[57] = -1000000;
    lh[58] = -1000000;
    lh[59] = -1000000;
    lh[60] = -1000000;
    lh[62] = 0.5;
    lh[63] = -1000000;
    lh[64] = -1000000;
    lh[65] = -1000000;
    lh[66] = -1000000;
    lh[67] = -1000000;
    lh[68] = -1000000;
    lh[70] = 0.5;
    lh[71] = -1000000;
    lh[72] = -1000000;
    lh[73] = -1000000;
    lh[74] = -1000000;
    lh[75] = -1000000;
    lh[76] = -1000000;
    lh[78] = 0.5;
    lh[79] = -1000000;
    lh[80] = -1000000;
    lh[81] = -1000000;
    lh[82] = -1000000;
    lh[83] = -1000000;
    lh[84] = -1000000;
    lh[86] = -1.5;
    lh[87] = -0.45;
    lh[92] = -1000000;
    lh[93] = -1000000;
    lh[94] = 0.5;
    lh[95] = -1000000;
    lh[96] = -1000000;
    lh[97] = -1000000;
    lh[98] = -1000000;
    lh[99] = -1000000;
    lh[100] = -1000000;
    lh[102] = 0.5;
    lh[103] = -1000000;
    lh[104] = -1000000;
    lh[105] = -1000000;
    lh[106] = -1000000;
    lh[107] = -1000000;
    lh[108] = -1000000;
    lh[110] = 0.5;
    lh[111] = -1000000;
    lh[112] = -1000000;
    lh[113] = -1000000;
    lh[114] = -1000000;
    lh[115] = -1000000;
    lh[116] = -1000000;
    lh[118] = 0.5;
    lh[119] = -1000000;
    lh[120] = -1000000;
    lh[121] = -1000000;
    lh[122] = -1000000;
    lh[123] = -1000000;
    lh[124] = -1000000;
    lh[126] = 0.5;
    lh[127] = -1000000;
    lh[128] = -1000000;
    lh[129] = -1000000;
    lh[130] = -1000000;
    lh[131] = -1000000;
    lh[132] = -1000000;
    lh[134] = 0.5;
    lh[135] = -1000000;
    lh[136] = -1000000;
    lh[137] = -1000000;
    lh[138] = -1000000;
    lh[139] = -1000000;
    lh[140] = -1000000;
    lh[142] = 0.5;
    lh[143] = -1000000;
    lh[144] = -1000000;
    lh[145] = -1000000;
    lh[146] = -1000000;
    lh[147] = -1000000;
    lh[148] = -1000000;
    lh[150] = 0.5;
    lh[151] = -1000000;
    lh[152] = -1000000;
    lh[153] = -1000000;
    lh[154] = -1000000;
    lh[155] = -1000000;
    lh[156] = -1000000;
    lh[158] = 0.5;
    lh[159] = -1000000;
    lh[160] = -1000000;
    lh[161] = -1000000;
    lh[162] = -1000000;
    lh[163] = -1000000;
    lh[164] = -1000000;
    lh[166] = 0.5;
    lh[167] = -1000000;
    lh[168] = -1000000;
    lh[169] = -1000000;
    lh[170] = -1000000;
    lh[171] = -1000000;
    lh[172] = -1000000;
    lh[174] = -1.5;
    lh[175] = -0.45;
    uh[0] = 0.09;
    uh[1] = 0.09;
    uh[2] = 0.09;
    uh[3] = 0.09;
    uh[6] = 1;
    uh[13] = 1000000;
    uh[14] = 1;
    uh[21] = 1000000;
    uh[22] = 1;
    uh[29] = 1000000;
    uh[30] = 1;
    uh[37] = 1000000;
    uh[38] = 1;
    uh[45] = 1000000;
    uh[46] = 1;
    uh[53] = 1000000;
    uh[54] = 1;
    uh[61] = 1000000;
    uh[62] = 1;
    uh[69] = 1000000;
    uh[70] = 1;
    uh[77] = 1000000;
    uh[78] = 1;
    uh[85] = 1000000;
    uh[86] = 1.5;
    uh[87] = 0.45;
    uh[88] = 0.09;
    uh[89] = 0.09;
    uh[90] = 0.09;
    uh[91] = 0.09;
    uh[94] = 1;
    uh[101] = 1000000;
    uh[102] = 1;
    uh[109] = 1000000;
    uh[110] = 1;
    uh[117] = 1000000;
    uh[118] = 1;
    uh[125] = 1000000;
    uh[126] = 1;
    uh[133] = 1000000;
    uh[134] = 1;
    uh[141] = 1000000;
    uh[142] = 1;
    uh[149] = 1000000;
    uh[150] = 1;
    uh[157] = 1000000;
    uh[158] = 1;
    uh[165] = 1000000;
    uh[166] = 1;
    uh[173] = 1000000;
    uh[174] = 1.5;
    uh[175] = 0.45;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lh", lh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "uh", uh);
    }
    free(luh);







    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "idxsbx", idxsbx);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "lsbx", lsbx);
    // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, 0, "usbx", usbx);

    // soft bounds on x
    int* idxsbx = malloc(NSBX * sizeof(int));
    idxsbx[0] = 0;
    idxsbx[1] = 2;

    double* lusbx = calloc(2*NSBX, sizeof(double));
    double* lsbx = lusbx;
    double* usbx = lusbx + NSBX;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxsbx", idxsbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lsbx", lsbx);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "usbx", usbx);
    }
    free(idxsbx);
    free(lusbx);


    // set up soft bounds for nonlinear constraints
    int* idxsh = malloc(NSH * sizeof(int));
    idxsh[0] = 0;
    idxsh[1] = 1;
    idxsh[2] = 2;
    idxsh[3] = 3;
    idxsh[4] = 7;
    idxsh[5] = 8;
    idxsh[6] = 9;
    idxsh[7] = 10;
    idxsh[8] = 11;
    idxsh[9] = 12;
    idxsh[10] = 15;
    idxsh[11] = 16;
    idxsh[12] = 17;
    idxsh[13] = 18;
    idxsh[14] = 19;
    idxsh[15] = 20;
    idxsh[16] = 23;
    idxsh[17] = 24;
    idxsh[18] = 25;
    idxsh[19] = 26;
    idxsh[20] = 27;
    idxsh[21] = 28;
    idxsh[22] = 31;
    idxsh[23] = 32;
    idxsh[24] = 33;
    idxsh[25] = 34;
    idxsh[26] = 35;
    idxsh[27] = 36;
    idxsh[28] = 39;
    idxsh[29] = 40;
    idxsh[30] = 41;
    idxsh[31] = 42;
    idxsh[32] = 43;
    idxsh[33] = 44;
    idxsh[34] = 47;
    idxsh[35] = 48;
    idxsh[36] = 49;
    idxsh[37] = 50;
    idxsh[38] = 51;
    idxsh[39] = 52;
    idxsh[40] = 55;
    idxsh[41] = 56;
    idxsh[42] = 57;
    idxsh[43] = 58;
    idxsh[44] = 59;
    idxsh[45] = 60;
    idxsh[46] = 63;
    idxsh[47] = 64;
    idxsh[48] = 65;
    idxsh[49] = 66;
    idxsh[50] = 67;
    idxsh[51] = 68;
    idxsh[52] = 71;
    idxsh[53] = 72;
    idxsh[54] = 73;
    idxsh[55] = 74;
    idxsh[56] = 75;
    idxsh[57] = 76;
    idxsh[58] = 79;
    idxsh[59] = 80;
    idxsh[60] = 81;
    idxsh[61] = 82;
    idxsh[62] = 83;
    idxsh[63] = 84;
    idxsh[64] = 86;
    idxsh[65] = 87;
    idxsh[66] = 88;
    idxsh[67] = 89;
    idxsh[68] = 90;
    idxsh[69] = 91;
    idxsh[70] = 95;
    idxsh[71] = 96;
    idxsh[72] = 97;
    idxsh[73] = 98;
    idxsh[74] = 99;
    idxsh[75] = 100;
    idxsh[76] = 103;
    idxsh[77] = 104;
    idxsh[78] = 105;
    idxsh[79] = 106;
    idxsh[80] = 107;
    idxsh[81] = 108;
    idxsh[82] = 111;
    idxsh[83] = 112;
    idxsh[84] = 113;
    idxsh[85] = 114;
    idxsh[86] = 115;
    idxsh[87] = 116;
    idxsh[88] = 119;
    idxsh[89] = 120;
    idxsh[90] = 121;
    idxsh[91] = 122;
    idxsh[92] = 123;
    idxsh[93] = 124;
    idxsh[94] = 127;
    idxsh[95] = 128;
    idxsh[96] = 129;
    idxsh[97] = 130;
    idxsh[98] = 131;
    idxsh[99] = 132;
    idxsh[100] = 135;
    idxsh[101] = 136;
    idxsh[102] = 137;
    idxsh[103] = 138;
    idxsh[104] = 139;
    idxsh[105] = 140;
    idxsh[106] = 143;
    idxsh[107] = 144;
    idxsh[108] = 145;
    idxsh[109] = 146;
    idxsh[110] = 147;
    idxsh[111] = 148;
    idxsh[112] = 151;
    idxsh[113] = 152;
    idxsh[114] = 153;
    idxsh[115] = 154;
    idxsh[116] = 155;
    idxsh[117] = 156;
    idxsh[118] = 159;
    idxsh[119] = 160;
    idxsh[120] = 161;
    idxsh[121] = 162;
    idxsh[122] = 163;
    idxsh[123] = 164;
    idxsh[124] = 167;
    idxsh[125] = 168;
    idxsh[126] = 169;
    idxsh[127] = 170;
    idxsh[128] = 171;
    idxsh[129] = 172;
    idxsh[130] = 174;
    idxsh[131] = 175;
    double* lush = calloc(2*NSH, sizeof(double));
    double* lsh = lush;
    double* ush = lush + NSH;

    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "idxsh", idxsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "lsh", lsh);
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, nlp_out, i, "ush", ush);
    }
    free(idxsh);
    free(lush);



    /* terminal constraints */




















}

// this function only sets external functions, numerical values are set in ocp_lipm_opti_pessi_acados_create_setup_nlp_in_numerical_values
void ocp_lipm_opti_pessi_acados_create_setup_nlp_in(ocp_lipm_opti_pessi_solver_capsule* capsule, const int N)
{
    assert(N == capsule->nlp_solver_plan->N);
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;

    /************************************************
    *  nlp_in
    ************************************************/
    ocp_nlp_in * nlp_in = capsule->nlp_in;
    /************************************************
    *  nlp_out
    ************************************************/
    ocp_nlp_out * nlp_out = capsule->nlp_out;


    /**** Dynamics ****/
    for (int i = 0; i < N; i++)
    {
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun", &capsule->discr_dyn_phi_fun[i]);
        ocp_nlp_dynamics_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "disc_dyn_fun_jac",
                                   &capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        
        
    }

    /**** Cost ****/
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun", &capsule->cost_y_0_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nls_y_fun_jac", &capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun", &capsule->cost_y_fun[i-1]);
        ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nls_y_fun_jac", &capsule->cost_y_fun_jac_ut_xt[i-1]);
    }
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun", &capsule->cost_y_e_fun);
    ocp_nlp_cost_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, N, "nls_y_fun_jac", &capsule->cost_y_e_fun_jac_ut_xt);

    /**** Constraints ****/

    // bounds for initial stage

    // set up nonlinear constraints for initial stage
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun_jac", &capsule->nl_constr_h_0_fun_jac);
    ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, 0, "nl_constr_h_fun", &capsule->nl_constr_h_0_fun);
    
    
    
    


    // set up nonlinear constraints for stage 1 to N-1
    for (int i = 1; i < N; i++)
    {
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun_jac",
                                      &capsule->nl_constr_h_fun_jac[i-1]);
        ocp_nlp_constraints_model_set_external_param_fun(nlp_config, nlp_dims, nlp_in, i, "nl_constr_h_fun",
                                      &capsule->nl_constr_h_fun[i-1]);
        
        
      
        
    }



    /* terminal constraints */
}


static void ocp_lipm_opti_pessi_acados_create_set_opts(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    void *nlp_opts = capsule->nlp_opts;

    /************************************************
    *  opts
    ************************************************/



    int fixed_hess = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "fixed_hess", &fixed_hess);
    double globalization_alpha_min = 0.05;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_alpha_min", &globalization_alpha_min);

    double globalization_alpha_reduction = 0.7;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "globalization_alpha_reduction", &globalization_alpha_reduction);



    int globalization_line_search_use_sufficient_descent = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_line_search_use_sufficient_descent", &globalization_line_search_use_sufficient_descent);

    int globalization_use_SOC = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_use_SOC", &globalization_use_SOC);

    double globalization_eps_sufficient_descent = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_eps_sufficient_descent", &globalization_eps_sufficient_descent);

    int with_solution_sens_wrt_params_forw = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params_forw", &with_solution_sens_wrt_params_forw);

    int with_solution_sens_wrt_params_adj = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_solution_sens_wrt_params_adj", &with_solution_sens_wrt_params_adj);

    int with_value_sens_wrt_params = false;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "with_value_sens_wrt_params", &with_value_sens_wrt_params);

    double solution_sens_qp_t_lam_min = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "solution_sens_qp_t_lam_min", &solution_sens_qp_t_lam_min);

    int globalization_full_step_dual = 0;
    ocp_nlp_solver_opts_set(nlp_config, capsule->nlp_opts, "globalization_full_step_dual", &globalization_full_step_dual);

    double levenberg_marquardt = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "levenberg_marquardt", &levenberg_marquardt);

    /* options QP solver */
    int qp_solver_cond_N;const int qp_solver_cond_N_ori = 6;
    qp_solver_cond_N = N < qp_solver_cond_N_ori ? N : qp_solver_cond_N_ori; // use the minimum value here
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    int nlp_solver_ext_qp_res = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "ext_qp_res", &nlp_solver_ext_qp_res);

    bool store_iterates = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "store_iterates", &store_iterates);
    int log_primal_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_primal_step_norm", &log_primal_step_norm);

    int log_dual_step_norm = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "log_dual_step_norm", &log_dual_step_norm);

    double nlp_solver_tol_min_step_norm = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_min_step_norm", &nlp_solver_tol_min_step_norm);
    // set HPIPM mode: should be done before setting other QP solver options
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_hpipm_mode", "BALANCE");



    int qp_solver_t0_init = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_t0_init", &qp_solver_t0_init);




    // set SQP specific options
    double nlp_solver_tol_stat = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_stat", &nlp_solver_tol_stat);

    double nlp_solver_tol_eq = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_eq", &nlp_solver_tol_eq);

    double nlp_solver_tol_ineq = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_ineq", &nlp_solver_tol_ineq);

    double nlp_solver_tol_comp = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "tol_comp", &nlp_solver_tol_comp);

    int nlp_solver_max_iter = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "max_iter", &nlp_solver_max_iter);

    // set options for adaptive Levenberg-Marquardt Update
    bool with_adaptive_levenberg_marquardt = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_adaptive_levenberg_marquardt", &with_adaptive_levenberg_marquardt);

    double adaptive_levenberg_marquardt_lam = 5;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_lam", &adaptive_levenberg_marquardt_lam);

    double adaptive_levenberg_marquardt_mu_min = 0.0000000000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu_min", &adaptive_levenberg_marquardt_mu_min);

    double adaptive_levenberg_marquardt_mu0 = 0.001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_mu0", &adaptive_levenberg_marquardt_mu0);

    double adaptive_levenberg_marquardt_obj_scalar = 2;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "adaptive_levenberg_marquardt_obj_scalar", &adaptive_levenberg_marquardt_obj_scalar);

    bool eval_residual_at_max_iter = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "eval_residual_at_max_iter", &eval_residual_at_max_iter);

    // QP scaling
    double qpscaling_ub_max_abs_eig = 100000;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_ub_max_abs_eig", &qpscaling_ub_max_abs_eig);

    double qpscaling_lb_norm_inf_grad_obj = 0.0001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_lb_norm_inf_grad_obj", &qpscaling_lb_norm_inf_grad_obj);

    qpscaling_scale_objective_type qpscaling_scale_objective = NO_OBJECTIVE_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_objective", &qpscaling_scale_objective);

    ocp_nlp_qpscaling_constraint_type qpscaling_scale_constraints = NO_CONSTRAINT_SCALING;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qpscaling_scale_constraints", &qpscaling_scale_constraints);

    // NLP QP tol strategy
    ocp_nlp_qp_tol_strategy_t nlp_qp_tol_strategy = FIXED_QP_TOL;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_strategy", &nlp_qp_tol_strategy);

    double nlp_qp_tol_reduction_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_reduction_factor", &nlp_qp_tol_reduction_factor);

    double nlp_qp_tol_safety_factor = 0.1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_safety_factor", &nlp_qp_tol_safety_factor);

    double nlp_qp_tol_min_stat = 0.000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_stat", &nlp_qp_tol_min_stat);

    double nlp_qp_tol_min_eq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_eq", &nlp_qp_tol_min_eq);

    double nlp_qp_tol_min_ineq = 0.0000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_ineq", &nlp_qp_tol_min_ineq);

    double nlp_qp_tol_min_comp = 0.00000000001;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "nlp_qp_tol_min_comp", &nlp_qp_tol_min_comp);

    bool with_anderson_acceleration = false;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "with_anderson_acceleration", &with_anderson_acceleration);

    double anderson_activation_threshold = 10;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "anderson_activation_threshold", &anderson_activation_threshold);

    int qp_solver_iter_max = 50;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_iter_max", &qp_solver_iter_max);



    int print_level = 0;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "print_level", &print_level);
    int qp_solver_cond_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_cond_ric_alg", &qp_solver_cond_ric_alg);

    int qp_solver_ric_alg = 1;
    ocp_nlp_solver_opts_set(nlp_config, nlp_opts, "qp_ric_alg", &qp_solver_ric_alg);


    int ext_cost_num_hess = 0;
}


/**
 * Internal function for ocp_lipm_opti_pessi_acados_create: step 7
 */
void ocp_lipm_opti_pessi_acados_set_nlp_out(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;

    // initialize primal solution
    double* xu0 = calloc(NX+NU, sizeof(double));
    double* x0 = xu0;

    // initialize with x0
    x0[0] = -0.8;
    x0[1] = -0.5;
    x0[6] = -0.5593;
    x0[7] = -0.634;
    x0[8] = -1.0407;
    x0[9] = -0.366;
    x0[11] = -0.8;
    x0[12] = -0.5;
    x0[17] = -0.5593;
    x0[18] = -0.634;
    x0[19] = -1.0407;
    x0[20] = -0.366;


    double* u0 = xu0 + NX;

    for (int i = 0; i < N; i++)
    {
        // x0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", x0);
        // u0
        ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "u", u0);
    }
    ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, N, "x", x0);
    free(xu0);
}


/**
 * Internal function for ocp_lipm_opti_pessi_acados_create: step 9
 */
int ocp_lipm_opti_pessi_acados_create_precompute(ocp_lipm_opti_pessi_solver_capsule* capsule) {
    int status = ocp_nlp_precompute(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    if (status != ACADOS_SUCCESS) {
        printf("\nocp_nlp_precompute failed!\n\n");
        exit(1);
    }

    return status;
}


int ocp_lipm_opti_pessi_acados_create_with_discretization(ocp_lipm_opti_pessi_solver_capsule* capsule, int N, double* new_time_steps)
{
    // If N does not match the number of shooting intervals used for code generation, new_time_steps must be given.
    if (N != OCP_LIPM_OPTI_PESSI_N && !new_time_steps) {
        fprintf(stderr, "ocp_lipm_opti_pessi_acados_create_with_discretization: new_time_steps is NULL " \
            "but the number of shooting intervals (= %d) differs from the number of " \
            "shooting intervals (= %d) during code generation! Please provide a new vector of time_stamps!\n", \
             N, OCP_LIPM_OPTI_PESSI_N);
        return 1;
    }

    // number of expected runtime parameters
    capsule->nlp_np = NP;

    // 1) create and set nlp_solver_plan; create nlp_config
    capsule->nlp_solver_plan = ocp_nlp_plan_create(N);
    ocp_lipm_opti_pessi_acados_create_set_plan(capsule->nlp_solver_plan, N);
    capsule->nlp_config = ocp_nlp_config_create(*capsule->nlp_solver_plan);

    // 2) create and set dimensions
    capsule->nlp_dims = ocp_lipm_opti_pessi_acados_create_setup_dimensions(capsule);

    // 3) create and set nlp_opts
    capsule->nlp_opts = ocp_nlp_solver_opts_create(capsule->nlp_config, capsule->nlp_dims);
    ocp_lipm_opti_pessi_acados_create_set_opts(capsule);

    // 4) create and set nlp_out
    // 4.1) nlp_out
    capsule->nlp_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    // 4.2) sens_out
    capsule->sens_out = ocp_nlp_out_create(capsule->nlp_config, capsule->nlp_dims);
    ocp_lipm_opti_pessi_acados_set_nlp_out(capsule);

    // 5) create nlp_in
    capsule->nlp_in = ocp_nlp_in_create(capsule->nlp_config, capsule->nlp_dims);

    // 6) setup functions, nlp_in and default parameters
    ocp_lipm_opti_pessi_acados_create_setup_functions(capsule);
    ocp_lipm_opti_pessi_acados_create_setup_nlp_in(capsule, N);
    ocp_lipm_opti_pessi_acados_create_setup_nlp_in_numerical_values(capsule, N, new_time_steps);
    ocp_lipm_opti_pessi_acados_create_set_default_parameters(capsule);

    // 7) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);


    // 8) do precomputations
    int status = ocp_lipm_opti_pessi_acados_create_precompute(capsule);

    return status;
}

/**
 * This function is for updating an already initialized solver with a different number of qp_cond_N. It is useful for code reuse after code export.
 */
int ocp_lipm_opti_pessi_acados_update_qp_solver_cond_N(ocp_lipm_opti_pessi_solver_capsule* capsule, int qp_solver_cond_N)
{
    // 1) destroy solver
    ocp_nlp_solver_destroy(capsule->nlp_solver);

    // 2) set new value for "qp_cond_N"
    const int N = capsule->nlp_solver_plan->N;
    if(qp_solver_cond_N > N)
        printf("Warning: qp_solver_cond_N = %d > N = %d\n", qp_solver_cond_N, N);
    ocp_nlp_solver_opts_set(capsule->nlp_config, capsule->nlp_opts, "qp_cond_N", &qp_solver_cond_N);

    // 3) continue with the remaining steps from ocp_lipm_opti_pessi_acados_create_with_discretization(...):
    // -> 8) create solver
    capsule->nlp_solver = ocp_nlp_solver_create(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_opts, capsule->nlp_in);

    // -> 9) do precomputations
    int status = ocp_lipm_opti_pessi_acados_create_precompute(capsule);
    return status;
}


int ocp_lipm_opti_pessi_acados_reset(ocp_lipm_opti_pessi_solver_capsule* capsule, int reset_qp_solver_mem, int reset_numerical_values, int reset_solver_options, int reset_x_to_x0_bar)
{
    // set initialization to all zeros
    const int N = capsule->nlp_solver_plan->N;
    ocp_nlp_config* nlp_config = capsule->nlp_config;
    ocp_nlp_dims* nlp_dims = capsule->nlp_dims;
    ocp_nlp_out* nlp_out = capsule->nlp_out;
    ocp_nlp_in* nlp_in = capsule->nlp_in;
    ocp_nlp_solver* nlp_solver = capsule->nlp_solver;

    // sets primal and dual iterates to zero
    ocp_nlp_out_set_values_to_zero(nlp_config, nlp_dims, nlp_out);

    // reset integrator memory
    ocp_nlp_solver_reset_integrator_memory(nlp_solver, nlp_in, nlp_out);
    // get qp_status: if NaN -> reset memory
    int qp_status;
    ocp_nlp_get(capsule->nlp_solver, "qp_status", &qp_status);
    if (reset_qp_solver_mem || (qp_status == 3))
    {
        // printf("\nin reset qp_status %d -> resetting QP memory\n", qp_status);
        ocp_nlp_solver_reset_qp_memory(nlp_solver, nlp_in, nlp_out);
    }

    if (reset_numerical_values)
    {
        // reset parameters to initial values
        ocp_lipm_opti_pessi_acados_create_set_default_parameters(capsule);

        // reset numerical values in nlp_in
        ocp_lipm_opti_pessi_acados_create_setup_nlp_in_numerical_values(capsule, N, NULL);
    }

    if (reset_solver_options)
    {
        // reset solver options to initial values
        ocp_lipm_opti_pessi_acados_create_set_opts(capsule);
    }

    if (reset_x_to_x0_bar)
    {double* buffer = calloc(NX, sizeof(double));
        ocp_nlp_constraints_model_get(nlp_config, nlp_dims, nlp_in, 0, "lbx", buffer);
        for (int i=0; i<N+1; i++)
        {
            ocp_nlp_out_set(nlp_config, nlp_dims, nlp_out, nlp_in, i, "x", buffer);
        }
        free(buffer);
    }
    return 0;
}




int ocp_lipm_opti_pessi_acados_update_params(ocp_lipm_opti_pessi_solver_capsule* capsule, int stage, double *p, int np)
{
    int solver_status = 0;

    int casadi_np = 46;
    if (casadi_np != np) {
        printf("acados_update_params: trying to set %i parameters for external functions."
            " External function has %i parameters. Exiting.\n", np, casadi_np);
        exit(1);
    }
    ocp_nlp_in_set(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, "parameter_values", p);

    return solver_status;
}


int ocp_lipm_opti_pessi_acados_update_params_sparse(ocp_lipm_opti_pessi_solver_capsule * capsule, int stage, int *idx, double *p, int n_update)
{
    ocp_nlp_in_set_params_sparse(capsule->nlp_config, capsule->nlp_dims, capsule->nlp_in, stage, idx, p, n_update);

    return 0;
}


int ocp_lipm_opti_pessi_acados_set_p_global_and_precompute_dependencies(ocp_lipm_opti_pessi_solver_capsule* capsule, double* data, int data_len)
{

    // printf("No global_data, ocp_lipm_opti_pessi_acados_set_p_global_and_precompute_dependencies does nothing.\n");
    return 0;
}




int ocp_lipm_opti_pessi_acados_solve(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    // solve NLP
    int solver_status = ocp_nlp_solve(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}



int ocp_lipm_opti_pessi_acados_setup_qp_matrices_and_factorize(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    int solver_status = ocp_nlp_setup_qp_matrices_and_factorize(capsule->nlp_solver, capsule->nlp_in, capsule->nlp_out);

    return solver_status;
}






int ocp_lipm_opti_pessi_acados_free(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    // before destroying, keep some info
    const int N = capsule->nlp_solver_plan->N;
    // free memory
    ocp_nlp_solver_opts_destroy(capsule->nlp_opts);
    ocp_nlp_in_destroy(capsule->nlp_in);
    ocp_nlp_out_destroy(capsule->nlp_out);
    ocp_nlp_out_destroy(capsule->sens_out);
    ocp_nlp_solver_destroy(capsule->nlp_solver);
    ocp_nlp_dims_destroy(capsule->nlp_dims);
    ocp_nlp_config_destroy(capsule->nlp_config);
    ocp_nlp_plan_destroy(capsule->nlp_solver_plan);

    /* free external function */
    // dynamics
    for (int i = 0; i < N; i++)
    {
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun[i]);
        external_function_external_param_casadi_free(&capsule->discr_dyn_phi_fun_jac_ut_xt[i]);
        
        

        
    }
    free(capsule->discr_dyn_phi_fun);
    free(capsule->discr_dyn_phi_fun_jac_ut_xt);
  
  
  

    // cost
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_0_fun_jac_ut_xt);
    for (int i = 0; i < N - 1; i++)
    {
        external_function_external_param_casadi_free(&capsule->cost_y_fun[i]);
        external_function_external_param_casadi_free(&capsule->cost_y_fun_jac_ut_xt[i]);
    }
    free(capsule->cost_y_fun);
    free(capsule->cost_y_fun_jac_ut_xt);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun);
    external_function_external_param_casadi_free(&capsule->cost_y_e_fun_jac_ut_xt);

    // constraints
    for (int i = 0; i < N-1; i++)
    {
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun_jac[i]);
        external_function_external_param_casadi_free(&capsule->nl_constr_h_fun[i]);
    }
    free(capsule->nl_constr_h_fun_jac);
    free(capsule->nl_constr_h_fun);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun_jac);
    external_function_external_param_casadi_free(&capsule->nl_constr_h_0_fun);



    return 0;
}


void ocp_lipm_opti_pessi_acados_print_stats(ocp_lipm_opti_pessi_solver_capsule* capsule)
{
    int nlp_iter, stat_m, stat_n, tmp_int;
    ocp_nlp_get(capsule->nlp_solver, "nlp_iter", &nlp_iter);
    ocp_nlp_get(capsule->nlp_solver, "stat_n", &stat_n);
    ocp_nlp_get(capsule->nlp_solver, "stat_m", &stat_m);


    int stat_n_max = 16;
    if (stat_n > stat_n_max)
    {
        printf("stat_n_max = %d is too small, increase it in the template!\n", stat_n_max);
        exit(1);
    }
    double stat[48];
    ocp_nlp_get(capsule->nlp_solver, "statistics", stat);

    int nrow = nlp_iter+1 < stat_m ? nlp_iter+1 : stat_m;


    printf("iter\tres_stat\tres_eq\t\tres_ineq\tres_comp\tqp_stat\tqp_iter\talpha");
    if (stat_n > 8)
        printf("\t\tqp_res_stat\tqp_res_eq\tqp_res_ineq\tqp_res_comp");
    printf("\n");
    for (int i = 0; i < nrow; i++)
    {
        for (int j = 0; j < stat_n + 1; j++)
        {
            if (j == 0 || j == 5 || j == 6)
            {
                tmp_int = (int) stat[i + j * nrow];
                printf("%d\t", tmp_int);
            }
            else
            {
                printf("%e\t", stat[i + j * nrow]);
            }
        }
        printf("\n");
    }
}

int ocp_lipm_opti_pessi_acados_custom_update(ocp_lipm_opti_pessi_solver_capsule* capsule, double* data, int data_len)
{
    (void)capsule;
    (void)data;
    (void)data_len;
    printf("\ndummy function that can be called in between solver calls to update parameters or numerical data efficiently in C.\n");
    printf("nothing set yet..\n");
    return 1;

}



ocp_nlp_in *ocp_lipm_opti_pessi_acados_get_nlp_in(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_in; }
ocp_nlp_out *ocp_lipm_opti_pessi_acados_get_nlp_out(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_out; }
ocp_nlp_out *ocp_lipm_opti_pessi_acados_get_sens_out(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->sens_out; }
ocp_nlp_solver *ocp_lipm_opti_pessi_acados_get_nlp_solver(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_solver; }
ocp_nlp_config *ocp_lipm_opti_pessi_acados_get_nlp_config(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_config; }
void *ocp_lipm_opti_pessi_acados_get_nlp_opts(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_opts; }
ocp_nlp_dims *ocp_lipm_opti_pessi_acados_get_nlp_dims(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_dims; }
ocp_nlp_plan_t *ocp_lipm_opti_pessi_acados_get_nlp_plan(ocp_lipm_opti_pessi_solver_capsule* capsule) { return capsule->nlp_solver_plan; }
