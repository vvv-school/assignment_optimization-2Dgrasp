// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#ifndef SOLVER_H
#define SOLVER_H

#include <vector>
#include <IpTNLP.hpp>
#include "problem.h"

namespace problem_ns {

/**
 * NLP API.
 */
class Grasp : public Ipopt::TNLP
{
protected:
    const Problem &problem;
    std::vector<Force> result;

    /***************************************************/
    bool get_nlp_info(Ipopt::Index &n, Ipopt::Index &m,
                      Ipopt::Index &nnz_jac_g,
                      Ipopt::Index &nnz_h_lag,
                      IndexStyleEnum &index_style) override;

    /***************************************************/
    bool get_bounds_info(Ipopt::Index n, Ipopt::Number *x_l,
                         Ipopt::Number *x_u, Ipopt::Index m,
                         Ipopt::Number *g_l, Ipopt::Number *g_u) override;

    /***************************************************/
    bool get_starting_point(Ipopt::Index n, bool init_x,
                            Ipopt::Number *x, bool init_z,
                            Ipopt::Number *z_L, Ipopt::Number *z_U,
                            Ipopt::Index m, bool init_lambda,
                            Ipopt::Number *lambda) override;

    /***************************************************/
    bool eval_f(Ipopt::Index n, const Ipopt::Number *x,
                bool new_x, Ipopt::Number &obj_value) override;

    /***************************************************/
    bool eval_grad_f(Ipopt::Index n, const Ipopt::Number *x,
                     bool new_x, Ipopt::Number *grad_f) override;

    /***************************************************/
    bool eval_g(Ipopt::Index n, const Ipopt::Number *x,
                bool new_x, Ipopt::Index m, Ipopt::Number *g) override;

    /***************************************************/
    bool eval_jac_g(Ipopt::Index n, const Ipopt::Number *x,
                    bool new_x, Ipopt::Index m, Ipopt::Index nele_jac,
                    Ipopt::Index *iRow, Ipopt::Index *jCol,
                    Ipopt::Number *values) override;

    /***************************************************/
    bool eval_h(Ipopt::Index n, const Ipopt::Number *x, bool new_x,
                Ipopt::Number obj_factor, Ipopt::Index m, const Ipopt::Number *lambda,
                bool new_lambda, Ipopt::Index nele_hess, Ipopt::Index *iRow,
                Ipopt::Index *jCol, Ipopt::Number *values) override
    {
        return true;
    }

    /***************************************************/
    void finalize_solution(Ipopt::SolverReturn status,
                           Ipopt::Index n, const Ipopt::Number *x,
                           const Ipopt::Number *z_L,
                           const Ipopt::Number *z_U, Ipopt::Index m,
                           const Ipopt::Number *g, const Ipopt::Number *lambda,
                           Ipopt::Number obj_value, const Ipopt::IpoptData *ip_data,
                           Ipopt::IpoptCalculatedQuantities *ip_cq) override;

public:
    /***************************************************/
    Grasp(const Problem &problem_) : problem(problem_), result(2) { }

    /***************************************************/
    const std::vector<Force>& get_result() const
    {
        return result;
    }
};

/**
 * Solver API.
 */
class Solver
{
public:
   /**
    * Solve the problem.
    * @param problem to solve.
    * @param verbose to enable verbosity.
    * @return a vector containing the applied forces.
    */
    static std::vector<Force> solve(const Problem& problem,
                                    const bool verbose=true);
};

}

#endif
