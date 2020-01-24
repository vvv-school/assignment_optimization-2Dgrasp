// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#include <cstdlib>
#include <memory>
#include <utility>
#include <string>
#include <sstream>
#include <vector>
#include <numeric>
#include <cmath>
#include <iostream>
#include <fstream>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Value.h>
#include <yarp/sig/Vector.h>
#include <yarp/math/Math.h>
#include "problem.h"
#include "solver.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::sig;
using namespace yarp::math;
using namespace problem_ns;

/***************************************************/
string forceToString(shared_ptr<Problem> problem,
                     const Force& F)
{
    ostringstream ss;
    ss << problem->get_P(F.t).toString() << " "
       << (F.fn*problem->get_N(F.t)).toString() << " "
       << (F.ft*problem->get_T(F.t)).toString();
    return ss.str();
}

/***************************************************/
int main(int argc, char* argv[])
{
    ResourceFinder rf;
    rf.configure(argc,argv);
    if (!rf.check("shape")) {
        cerr << "Please provide \"--shape [circle|patch]\"" << endl;
        return EXIT_FAILURE;
    }
    auto type=rf.find("shape").asString();
    if ((type!="circle") && (type!="patch")) {
        cerr << "Unrecognized shape \"" << type << "\"" << endl;
        return EXIT_FAILURE;
    }

    auto problem=Problem::generate();
    if (type=="circle") {
        auto F=problem->get_F(); F.ft=0.;
        problem->configure(vector<double>({.0,.0,.0,.0}),problem->get_friction(),F);
    }
    auto forces=Solver::solve(*problem);
    auto F_M=problem->compute_newton_law(forces);

    cout.precision(5); cout << fixed;
    cout << "F = " << F_M.first.toString(5,5) << endl;
    cout << "M = " << F_M.second << endl;
    if (!problem->check_no_slippage(forces)) {
        cerr << "Solved forces are causing slippage!" << endl;
    }

    vector<double> tval(1000);
    iota(begin(tval),end(tval),0.);

    ofstream fout("problem.out");

    fout << tval.size() << endl;
    for (auto &t:tval) {
        t*=2.*M_PI/tval.size();
        fout << t << " " << problem->get_P(t).toString(5,5) << endl;
    }
    fout << problem->get_friction() << endl;
    fout << forceToString(problem,problem->get_F()) << " " << problem->get_N(problem->get_F().t).toString(5,5) << endl;
    fout << forceToString(problem,forces[0]) << " " << problem->get_N(forces[0].t).toString(5,5) << endl;
    fout << forceToString(problem,forces[1]) << " " << problem->get_N(forces[1].t).toString(5,5) << endl;
    fout << problem->get_COM().toString(5,5) << endl;
    fout << F_M.first.toString(5,5) << endl;
    fout << F_M.second<< endl;
    
    fout.close();
    return EXIT_SUCCESS;
}
