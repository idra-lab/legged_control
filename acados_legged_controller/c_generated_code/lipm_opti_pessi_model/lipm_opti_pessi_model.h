/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * Licensed under the 2-Clause BSD License.
 */

#ifndef lipm_opti_pessi_MODEL
#define lipm_opti_pessi_MODEL

#ifdef __cplusplus
extern "C" {
#endif




int lipm_opti_pessi_dyn_disc_phi_fun(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_dyn_disc_phi_fun_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_dyn_disc_phi_fun_sparsity_in(int);
const int *lipm_opti_pessi_dyn_disc_phi_fun_sparsity_out(int);
int lipm_opti_pessi_dyn_disc_phi_fun_n_in(void);
int lipm_opti_pessi_dyn_disc_phi_fun_n_out(void);

int lipm_opti_pessi_dyn_disc_phi_fun_jac(const real_t** arg, real_t** res, int* iw, real_t* w, void *mem);
int lipm_opti_pessi_dyn_disc_phi_fun_jac_work(int *, int *, int *, int *);
const int *lipm_opti_pessi_dyn_disc_phi_fun_jac_sparsity_in(int);
const int *lipm_opti_pessi_dyn_disc_phi_fun_jac_sparsity_out(int);
int lipm_opti_pessi_dyn_disc_phi_fun_jac_n_in(void);
int lipm_opti_pessi_dyn_disc_phi_fun_jac_n_out(void);












#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // lipm_opti_pessi_MODEL
