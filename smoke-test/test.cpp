/*
 * Copyright (C) 2020 iCub Tech Facility - Istituto Italiano di Tecnologia
 * Author: Ugo Pattacini <ugo.pattacini@iit.it>
 * CopyPolicy: Released under the terms of the GNU GPL v3.0.
*/

#undef NDEBUG
#include <cassert>

#include <memory>
#include <utility>
#include <string>
#include <sstream>
#include <vector>
#include <cmath>
#include <numeric>
#include <robottestingframework/dll/Plugin.h>
#include <robottestingframework/TestAssert.h>
#include <yarp/robottestingframework/TestCase.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include <assignment_optimization-2Dgrasplib/problem.h>
#include <assignment_optimization-2Dgrasplib/solver.h>

using namespace std;
using namespace robottestingframework;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace problem_ns;

/**********************************************************************/
class TestAssignmentOptimization2DGrasp : public yarp::robottestingframework::TestCase
{
    // we duplicate code here instead of reusing the lib in order to avoid cheating 😉

    /******************************************************************/
    pair<Vector,double> compute_newton_law(const Problem& problem, const vector<Force>& forces) const
    {
        assert(forces.size()==2);
        auto F=problem.get_F();
        auto COM=problem.get_COM();
        auto Ftot=F.fn*problem.get_N(F.t)+forces[0].fn*problem.get_N(forces[0].t)+forces[1].fn*problem.get_N(forces[1].t)+
                  F.ft*problem.get_T(F.t)+forces[0].ft*problem.get_T(forces[0].t)+forces[1].ft*problem.get_T(forces[1].t);
        auto Ttot=(problem.get_P(F.t)-COM)[0]*(F.fn*problem.get_N(F.t)[1]+F.ft*problem.get_T(F.t)[1])-
                  (problem.get_P(F.t)-COM)[1]*(F.fn*problem.get_N(F.t)[0]+F.ft*problem.get_T(F.t)[0])+
                  (problem.get_P(forces[0].t)-COM)[0]*(forces[0].fn*problem.get_N(forces[0].t)[1]+forces[0].ft*problem.get_T(forces[0].t)[1])-
                  (problem.get_P(forces[0].t)-COM)[1]*(forces[0].fn*problem.get_N(forces[0].t)[0]+forces[0].ft*problem.get_T(forces[0].t)[0])+
                  (problem.get_P(forces[1].t)-COM)[0]*(forces[1].fn*problem.get_N(forces[1].t)[1]+forces[1].ft*problem.get_T(forces[1].t)[1])-
                  (problem.get_P(forces[1].t)-COM)[1]*(forces[1].fn*problem.get_N(forces[1].t)[0]+forces[1].ft*problem.get_T(forces[1].t)[0]);
        return make_pair(Ftot,Ttot);
    }

    /******************************************************************/
    bool check_no_slippage(const Problem& problem, const vector<Force>& forces) const
    {
        for (auto &f:forces) {
            if (fabs(f.ft)>problem.get_friction()*fabs(f.fn)) {
                return false;
            }
        }
        return true;
    }
    
    /******************************************************************/
    int assign_score(const int failures, const int N, const double M,
                     const double boost) const
    {
        auto perc=1.-(double)failures/(double)N;
        auto score=(int)ceil(M*perc);
        if (perc>=boost) {
            score<<=1;
        }
        return score;
    }

public:
    /******************************************************************/
    TestAssignmentOptimization2DGrasp() :
        yarp::robottestingframework::TestCase("TestAssignmentOptimization2DGrasp") { }

    /******************************************************************/
    virtual ~TestAssignmentOptimization2DGrasp() { }

    /******************************************************************/
    bool setup(Property& property) override { return true; }

    /******************************************************************/
    void run() override
    {
        double F_eps{.01}, T_eps{.01};
        int N{100}, score{0};

        vector<string> types{"circle","patch"};
        for (auto &type:types) {
            int F_fails{0}, T_fails{0}, slippage_fails{0};
            
            for (int n=0; n<N; n++) {
                ostringstream ss;
                ss.precision(5);
                ss << fixed;
                ss << "--- Run #" << n << " (" << type << "): ";

                auto problem=Problem::generate();
                if (type=="circle") {
                    auto F=problem->get_F(); F.ft=0.;
                    problem->configure(vector<double>({.0,.0,.0,.0}),problem->get_friction(),F);
                }
                auto forces=Solver::solve(*problem,false);
                auto F_T=compute_newton_law(*problem,forces);
                bool failure_detected{false};

                auto F=norm(F_T.first);
                if (F>F_eps) {
                    ss << "⚠ |F| = " << F << " > " << F_eps << "; ";
                    failure_detected=true;
                    F_fails++;
                }

                auto T=fabs(F_T.second);
                if (T>T_eps) {
                    ss << "⚠ |T| = " << T << " > " << T_eps << "; ";
                    failure_detected=true;
                    T_fails++;
                }

                if (!check_no_slippage(*problem,forces)) {
                    ss << "⚠ slippage detected!; ";
                    failure_detected=true;
                    slippage_fails++;
                }

                if (!failure_detected) {
                    ss << "✔";
                }
                ROBOTTESTINGFRAMEWORK_TEST_REPORT(ss.str());
            }

            auto M=4.;
            auto boost=(type=="circle"?1.1:.98);
            auto F_points=assign_score(F_fails,N,M,boost);
            auto T_points=assign_score(T_fails,N,M,boost);
            auto slippage_points=assign_score(slippage_fails,N,M,boost);
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("|F| < %g verified %d / %d ➡ %d points granted",F_eps,N-F_fails,N,F_points));
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("|T| < %g verified %d / %d ➡ %d points granted",T_eps,N-T_fails,N,T_points));
            ROBOTTESTINGFRAMEWORK_TEST_REPORT(Asserter::format("slippage checks successful %d / %d ➡ %d points granted",N-slippage_fails,N,slippage_points));
            score+=F_points+T_points+slippage_points;
        }
        ROBOTTESTINGFRAMEWORK_TEST_CHECK(score>0,Asserter::format("Total score = %d",score));
    }
};

ROBOTTESTINGFRAMEWORK_PREPARE_PLUGIN(TestAssignmentOptimization2DGrasp)
