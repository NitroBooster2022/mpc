/*
 * Copyright (c) The acados authors.
 *
 * This file is part of acados.
 *
 * The 2-Clause BSD License
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.;
 */

#ifndef ACADOS_SOLVER_park_H_
#define ACADOS_SOLVER_park_H_

#include "acados/utils/types.h"

#include "acados_c/ocp_nlp_interface.h"
#include "acados_c/external_function_interface.h"

#define PARK_NX     3
#define PARK_NZ     0
#define PARK_NU     2
#define PARK_NP     0
#define PARK_NBX    2
#define PARK_NBX0   3
#define PARK_NBU    2
#define PARK_NSBX   0
#define PARK_NSBU   0
#define PARK_NSH    0
#define PARK_NSG    0
#define PARK_NSPHI  0
#define PARK_NSHN   0
#define PARK_NSGN   0
#define PARK_NSPHIN 0
#define PARK_NSBXN  0
#define PARK_NS     0
#define PARK_NSN    0
#define PARK_NG     0
#define PARK_NBXN   0
#define PARK_NGN    0
#define PARK_NY0    5
#define PARK_NY     5
#define PARK_NYN    3
#define PARK_N      100
#define PARK_NH     0
#define PARK_NPHI   0
#define PARK_NHN    0
#define PARK_NPHIN  0
#define PARK_NR     0

#ifdef __cplusplus
extern "C" {
#endif


// ** capsule for solver data **
typedef struct park_solver_capsule
{
    // acados objects
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;
    ocp_nlp_out *sens_out;
    ocp_nlp_solver *nlp_solver;
    void *nlp_opts;
    ocp_nlp_plan_t *nlp_solver_plan;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;

    // number of expected runtime parameters
    unsigned int nlp_np;

    /* external functions */
    // dynamics

    external_function_param_casadi *forw_vde_casadi;
    external_function_param_casadi *expl_ode_fun;




    // cost






    // constraints




} park_solver_capsule;

ACADOS_SYMBOL_EXPORT park_solver_capsule * park_acados_create_capsule(void);
ACADOS_SYMBOL_EXPORT int park_acados_free_capsule(park_solver_capsule *capsule);

ACADOS_SYMBOL_EXPORT int park_acados_create(park_solver_capsule * capsule);

ACADOS_SYMBOL_EXPORT int park_acados_reset(park_solver_capsule* capsule, int reset_qp_solver_mem);

/**
 * Generic version of park_acados_create which allows to use a different number of shooting intervals than
 * the number used for code generation. If new_time_steps=NULL and n_time_steps matches the number used for code
 * generation, the time-steps from code generation is used.
 */
ACADOS_SYMBOL_EXPORT int park_acados_create_with_discretization(park_solver_capsule * capsule, int n_time_steps, double* new_time_steps);
/**
 * Update the time step vector. Number N must be identical to the currently set number of shooting nodes in the
 * nlp_solver_plan. Returns 0 if no error occurred and a otherwise a value other than 0.
 */
ACADOS_SYMBOL_EXPORT int park_acados_update_time_steps(park_solver_capsule * capsule, int N, double* new_time_steps);
/**
 * This function is used for updating an already initialized solver with a different number of qp_cond_N.
 */
ACADOS_SYMBOL_EXPORT int park_acados_update_qp_solver_cond_N(park_solver_capsule * capsule, int qp_solver_cond_N);
ACADOS_SYMBOL_EXPORT int park_acados_update_params(park_solver_capsule * capsule, int stage, double *value, int np);
ACADOS_SYMBOL_EXPORT int park_acados_update_params_sparse(park_solver_capsule * capsule, int stage, int *idx, double *p, int n_update);

ACADOS_SYMBOL_EXPORT int park_acados_solve(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int park_acados_free(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void park_acados_print_stats(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT int park_acados_custom_update(park_solver_capsule* capsule, double* data, int data_len);


ACADOS_SYMBOL_EXPORT ocp_nlp_in *park_acados_get_nlp_in(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *park_acados_get_nlp_out(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_out *park_acados_get_sens_out(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_solver *park_acados_get_nlp_solver(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_config *park_acados_get_nlp_config(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT void *park_acados_get_nlp_opts(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_dims *park_acados_get_nlp_dims(park_solver_capsule * capsule);
ACADOS_SYMBOL_EXPORT ocp_nlp_plan_t *park_acados_get_nlp_plan(park_solver_capsule * capsule);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif  // ACADOS_SOLVER_park_H_
