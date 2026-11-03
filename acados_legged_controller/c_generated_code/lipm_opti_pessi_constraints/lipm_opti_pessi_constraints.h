/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef lipm_opti_pessi_CONSTRAINTS
#define lipm_opti_pessi_CONSTRAINTS

#ifdef __cplusplus
extern "C" {
#endif









int lipm_opti_pessi_constr_h_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_constr_h_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_constr_h_fun_jac_uxt_zt_sparsity_in(int);
const int *lipm_opti_pessi_constr_h_fun_jac_uxt_zt_sparsity_out(int);
int lipm_opti_pessi_constr_h_fun_jac_uxt_zt_n_in(void);
int lipm_opti_pessi_constr_h_fun_jac_uxt_zt_n_out(void);

int lipm_opti_pessi_constr_h_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_constr_h_fun_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_constr_h_fun_sparsity_in(int);
const int *lipm_opti_pessi_constr_h_fun_sparsity_out(int);
int lipm_opti_pessi_constr_h_fun_n_in(void);
int lipm_opti_pessi_constr_h_fun_n_out(void);











int lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt_sparsity_in(int);
const int *lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt_sparsity_out(int);
int lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt_n_in(void);
int lipm_opti_pessi_constr_h_0_fun_jac_uxt_zt_n_out(void);

int lipm_opti_pessi_constr_h_0_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_constr_h_0_fun_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_constr_h_0_fun_sparsity_in(int);
const int *lipm_opti_pessi_constr_h_0_fun_sparsity_out(int);
int lipm_opti_pessi_constr_h_0_fun_n_in(void);
int lipm_opti_pessi_constr_h_0_fun_n_out(void);














#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // lipm_opti_pessi_CONSTRAINTS
