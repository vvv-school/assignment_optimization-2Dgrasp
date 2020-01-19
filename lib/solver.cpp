// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#undef NDEBUG
#include <assert.h>

#include <cmath>
#include <limits>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <IpIpoptApplication.hpp>
#include "solver.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace problem_ns;

/***************************************************/
bool Grasp::get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                         Ipopt::Index &nnz_jac_g,
                         Ipopt::Index &nnz_h_lag,
                         IndexStyleEnum &index_style)
{
    // FIX THE CODE
    n=0;
    m=0;
    nnz_jac_g=0;
    nnz_h_lag=0;
    index_style=TNLP::C_STYLE;
    return true;
}

/***************************************************/
bool Grasp::get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l,
                            Ipopt::Number *x_u, Ipopt::Index m,
                            Ipopt::Number *g_l, Ipopt::Number *g_u)
{
    // FILL IN THE CODE
    return true;
}

/***************************************************/
bool Grasp::get_starting_point(Ipopt::Index n, bool init_x,
                               Ipopt::Number *x, bool init_z,
                               Ipopt::Number *z_L, Ipopt::Number *z_U,
                               Ipopt::Index m, bool init_lambda,
                               Ipopt::Number *lambda)
{                                        
    // FILL IN THE CODE
    return true;
}

/***************************************************/
bool Grasp::eval_f(Ipopt::Index n, const Ipopt::Number *x,
                   bool new_x, Ipopt::Number &obj_value)
{
    // FILL IN THE CODE
    assert(!isnan(obj_value));
    return true;
}

/***************************************************/
bool Grasp::eval_grad_f(Ipopt::Index n, const Ipopt::Number *x,
                        bool new_x, Ipopt::Number *grad_f)
{
    // FILL IN THE CODE
    for (Ipopt::Index i=0; i<n; i++) {
        assert(!isnan(grad_f[i]));
    }
    return true;
}

/***************************************************/
bool Grasp::eval_g(Ipopt::Index n, const Ipopt::Number *x,
                   bool new_x, Ipopt::Index m, Ipopt::Number *g)
{
    // FILL IN THE CODE
    for (Ipopt::Index i=0; i<m; i++) {
        assert(!isnan(g[i]));
    }
    return true;
}

/***************************************************/
bool Grasp::eval_jac_g(Ipopt::Index n, const Ipopt::Number *x,
                       bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
                       Ipopt::Index *iRow, Ipopt::Index *jCol,
                       Ipopt::Number *values)
{
    // FILL IN THE CODE
    if (values==nullptr) {
    } else {
    }
    return true;
}

/***************************************************/
void Grasp::finalize_solution(Ipopt::SolverReturn status,
                              Ipopt::Index n, const Ipopt::Number *x,
                              const Ipopt::Number *z_L,
                              const Ipopt::Number *z_U, Ipopt::Index m,
                              const Ipopt::Number *g, const Ipopt::Number *lambda,
                              Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                              Ipopt::IpoptCalculatedQuantities *ip_cq)
{
    // FIX THE CODE
    result[0].t=0.;
    result[0].fn=0.;
    result[0].ft=0.;
    result[1].t=0.;
    result[1].fn=0.;
    result[1].ft=0.;
}

/***************************************************/
vector<Force> Solver::solve(const Problem& problem,
                            const bool verbose)
{
    Ipopt::SmartPtr<Ipopt::IpoptApplication> app=new Ipopt::IpoptApplication;
    app->Options()->SetNumericValue("tol",1e-6);
    app->Options()->SetNumericValue("constr_viol_tol",1e-3);
    app->Options()->SetIntegerValue("acceptable_iter",0);
    app->Options()->SetStringValue("mu_strategy","monotone");
    app->Options()->SetIntegerValue("max_iter",1000);
    app->Options()->SetStringValue("hessian_approximation","limited-memory");
    app->Options()->SetStringValue("derivative_test",verbose?"first-order":"none");
    app->Options()->SetIntegerValue("print_level",verbose?5:0);
    app->Initialize();

    Ipopt::SmartPtr<Grasp> nlp=new Grasp(problem);
    app->OptimizeTNLP(Ipopt::GetRawPtr(nlp));
    return nlp->get_result();
}
