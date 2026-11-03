/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */


#ifndef lipm_opti_pessi_COST
#define lipm_opti_pessi_COST

#ifdef __cplusplus
extern "C" {
#endif


// Cost at initial shooting node

int lipm_opti_pessi_cost_y_0_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_cost_y_0_fun_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_cost_y_0_fun_sparsity_in(int);
const int *lipm_opti_pessi_cost_y_0_fun_sparsity_out(int);
int lipm_opti_pessi_cost_y_0_fun_n_in(void);
int lipm_opti_pessi_cost_y_0_fun_n_out(void);

int lipm_opti_pessi_cost_y_0_fun_jac_ut_xt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_cost_y_0_fun_jac_ut_xt_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_cost_y_0_fun_jac_ut_xt_sparsity_in(int);
const int *lipm_opti_pessi_cost_y_0_fun_jac_ut_xt_sparsity_out(int);
int lipm_opti_pessi_cost_y_0_fun_jac_ut_xt_n_in(void);
int lipm_opti_pessi_cost_y_0_fun_jac_ut_xt_n_out(void);



// Cost at path shooting node

int lipm_opti_pessi_cost_y_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_cost_y_fun_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_cost_y_fun_sparsity_in(int);
const int *lipm_opti_pessi_cost_y_fun_sparsity_out(int);
int lipm_opti_pessi_cost_y_fun_n_in(void);
int lipm_opti_pessi_cost_y_fun_n_out(void);

int lipm_opti_pessi_cost_y_fun_jac_ut_xt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_cost_y_fun_jac_ut_xt_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_cost_y_fun_jac_ut_xt_sparsity_in(int);
const int *lipm_opti_pessi_cost_y_fun_jac_ut_xt_sparsity_out(int);
int lipm_opti_pessi_cost_y_fun_jac_ut_xt_n_in(void);
int lipm_opti_pessi_cost_y_fun_jac_ut_xt_n_out(void);



// Cost at terminal shooting node

int lipm_opti_pessi_cost_y_e_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_cost_y_e_fun_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_cost_y_e_fun_sparsity_in(int);
const int *lipm_opti_pessi_cost_y_e_fun_sparsity_out(int);
int lipm_opti_pessi_cost_y_e_fun_n_in(void);
int lipm_opti_pessi_cost_y_e_fun_n_out(void);

int lipm_opti_pessi_cost_y_e_fun_jac_ut_xt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_cost_y_e_fun_jac_ut_xt_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_cost_y_e_fun_jac_ut_xt_sparsity_in(int);
const int *lipm_opti_pessi_cost_y_e_fun_jac_ut_xt_sparsity_out(int);
int lipm_opti_pessi_cost_y_e_fun_jac_ut_xt_n_in(void);
int lipm_opti_pessi_cost_y_e_fun_jac_ut_xt_n_out(void);




#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // lipm_opti_pessi_COST
