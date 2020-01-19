// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#undef NDEBUG
#include <assert.h>

#include <algorithm>
#include <random>
#include <functional>
#include <yarp/math/Math.h>
#include <gsl/gsl_integration.h>
#include "problem.h"

using namespace std;
using namespace yarp::sig;
using namespace yarp::math;
using namespace problem_ns;

namespace problem_ns {

/***************************************************/
double integrand_M(double t, void* params)
{
    Problem *problem=static_cast<Problem*>(params);
    assert(problem);
    auto r2=norm2(problem->get_P(t));
    return (sqrt(r2)/2.);
}

double integrand_COMx(double t, void* params)
{
    Problem *problem=static_cast<Problem*>(params);
    assert(problem);
    auto r2=norm2(problem->get_P(t));
    return ((r2*cos(t))/3.);
} 

double integrand_COMy(double t, void* params)
{
    Problem *problem=static_cast<Problem*>(params);
    assert(problem);
    auto r2=norm2(problem->get_P(t));
    return ((r2*sin(t))/3.);
} 

}

/***************************************************/
bool Problem::configure(const vector<double> &ci,
                        const double friction,
                        const Force &F)
{
    configured=false;
    if ((ci.size()==this->ci.size()) &&
        (friction>=0.) && (friction<=1.)) {
        this->ci=ci;
        this->friction=friction;
        this->F=F;
        auto ft_max=this->friction*fabs(this->F.fn);
        this->F.ft=std::max(-ft_max,std::min(this->F.ft,ft_max));
        configured=true;
        COM=calc_COM();
    }
    return configured;
}

/***************************************************/
shared_ptr<Problem> Problem::generate()
{
    random_device rnd_device;
    mt19937 mersenne_engine(rnd_device());
    
    uniform_real_distribution<double> dist_ci(-.3,.3);
    uniform_real_distribution<double> dist_friction(.5,1.);
    uniform_real_distribution<double> dist_Ft(0.,2.*M_PI);
    uniform_real_distribution<double> dist_Ffn(.1,1.);
    
    auto gen_ci=bind(dist_ci,mersenne_engine);
    auto gen_friction=bind(dist_friction,mersenne_engine);
    auto gen_Ft=bind(dist_Ft,mersenne_engine);
    auto gen_Ffn=bind(dist_Ffn,mersenne_engine);
    
    vector<double> ci(4);
    std::generate(begin(ci),end(ci),gen_ci);
    auto friction=gen_friction();

    auto Ffn=gen_Ffn();	
    auto f=friction*Ffn;	
    uniform_real_distribution<double> dist_Fft(-f,f);	
    auto gen_Fft=bind(dist_Fft,mersenne_engine);	

    Force F{.t=gen_Ft(),.fn=Ffn,.ft=gen_Fft()};

    auto problem=shared_ptr<Problem>(new Problem);
    problem->configure(ci,friction,F);
    return problem;
}

/***************************************************/
double Problem::wrap_angle(const double t)
{
    auto tw=fmod(t,2.*M_PI);
    if (tw<0.) {
        tw+=2.*M_PI;
    }
    return tw;
}

/***************************************************/
size_t Problem::get_quadrant(const double t) const
{
    auto q=(size_t)floor(t/(M_PI/2.));
    q%=ti.size();
    return q;
}

/***************************************************/
tuple<double,size_t,double> Problem::calc_quantities(const double t) const
{
    auto q=get_quadrant(t);
    auto te=(t-ti[q])/si[q];
    auto r=1.+ci[q]*exp(-te*te);
    assert(!isnan(te) && !isnan(r));
    return make_tuple(r,q,te);
}

/***************************************************/
Vector Problem::calc_COM()
{
    size_t limit=10000;
    auto ws=gsl_integration_workspace_alloc(limit);
    auto integrand=shared_ptr<gsl_function>(new gsl_function);
    integrand->params=static_cast<void*>(this);

    double epsabs=.0001;
    double epsrel=.001;
    double abserr;

    double area;
    integrand->function=&integrand_M;
    gsl_integration_qag(integrand.get(),0.,2.*M_PI,epsabs,epsrel,limit,GSL_INTEG_GAUSS61,ws,&area,&abserr);

    Vector COM(2);
    integrand->function=&integrand_COMx;
    gsl_integration_qag(integrand.get(),0.,2.*M_PI,epsabs,epsrel,limit,GSL_INTEG_GAUSS61,ws,&COM[0],&abserr);

    integrand->function=&integrand_COMy;
    gsl_integration_qag(integrand.get(),0.,2.*M_PI,epsabs,epsrel,limit,GSL_INTEG_GAUSS61,ws,&COM[1],&abserr);

    gsl_integration_workspace_free(ws);

    COM/=area;
    assert(!isnan(COM[0]) && !isnan(COM[1]));
    return COM;
}

/***************************************************/
const vector<double>& Problem::get_ci() const
{
    assert(configured);
    return ci;
}

/***************************************************/
double Problem::get_friction() const
{
    assert(configured);
    return friction;
}

/***************************************************/
const Force& Problem::get_F() const
{
    assert(configured);
    return F;
}

/***************************************************/
const Vector& Problem::get_COM() const
{
    assert(configured);
    return COM;
}

/***************************************************/
Vector Problem::get_P(const double t) const
{
    assert(configured);
    auto tw=wrap_angle(t);
    auto qnt=calc_quantities(tw);
    auto &r=get<0>(qnt);
    Vector P(2);
    P[0]=r*cos(tw);
    P[1]=r*sin(tw);
    assert(!isnan(P[0]) && !isnan(P[1]));
    return P;
}

/***************************************************/
Vector Problem::get_dP(const double t) const
{
    assert(configured);
    auto tw=wrap_angle(t);
    auto qnt=calc_quantities(tw);
    auto &r=get<0>(qnt);
    auto &q=get<1>(qnt);
    auto &twe=get<2>(qnt);
    Vector dP(2);
    dP[0]=(r-1.)*(-2.*twe/si[q])*cos(tw)-r*sin(tw);
    dP[1]=(r-1.)*(-2.*twe/si[q])*sin(tw)+r*cos(tw);
    assert(!isnan(dP[0]) && !isnan(dP[1]));
    return dP;
}

/***************************************************/
Vector Problem::get_d2P(const double t) const
{
    assert(configured);
    auto tw=wrap_angle(t);
    auto qnt=calc_quantities(tw);
    auto &r=get<0>(qnt);
    auto &q=get<1>(qnt);
    auto &twe=get<2>(qnt);
    Vector d2P(2);
    d2P[0]=(r-1.)*4.*(twe/si[q])*(twe/si[q])*cos(tw)
           +(r-1.)*(-2./(si[q]*si[q]))*cos(tw)
           -(r-1.)*(-2.*twe/si[q])*sin(tw)
           -(r-1.)*(-2.*twe/si[q])*sin(tw)
           -r*cos(tw);
    d2P[1]=(r-1.)*4.*(twe/si[q])*(twe/si[q])*sin(tw)
           +(r-1.)*(-2./(si[q]*si[q]))*sin(tw)
           +(r-1.)*(-2.*twe/si[q])*cos(tw)
           +(r-1.)*(-2.*twe/si[q])*cos(tw)
           -r*sin(tw);
    assert(!isnan(d2P[0]) && !isnan(d2P[1]));
    return d2P;
}

/***************************************************/
Vector Problem::get_T(const double t) const
{
    return get_dP(t);
}

/***************************************************/
Vector Problem::get_dT(const double t) const
{
    return get_d2P(t);
}

/***************************************************/
Vector Problem::get_N(const double t) const
{
    auto dP=get_dP(t);
    Vector N(2);
    N[0]=-dP[1];
    N[1]=dP[0];
    return N;
}

/***************************************************/
Vector Problem::get_dN(const double t) const
{
    auto d2P=get_d2P(t);
    Vector dN(2);
    dN[0]=-d2P[1];
    dN[1]=d2P[0];
    return dN;
}

/***************************************************/
pair<Vector,double> Problem::compute_newton_laws(const vector<Force>& forces) const
{
    assert(forces.size()==2);

    // Newton 1st law (x,y axes)
    auto Ftot=F.fn*get_N(F.t)+forces[0].fn*get_N(forces[0].t)+forces[1].fn*get_N(forces[1].t)+
              F.ft*get_T(F.t)+forces[0].ft*get_T(forces[0].t)+forces[1].ft*get_T(forces[1].t);

    // Newton 2nd law (z-axis)
    auto Mtot=(get_P(F.t)-COM)[0]*(F.fn*get_N(F.t)[1]+F.ft*get_T(F.t)[1])-
              (get_P(F.t)-COM)[1]*(F.fn*get_N(F.t)[0]+F.ft*get_T(F.t)[0])+
              (get_P(forces[0].t)-COM)[0]*(forces[0].fn*get_N(forces[0].t)[1]+forces[0].ft*get_T(forces[0].t)[1])-
              (get_P(forces[0].t)-COM)[1]*(forces[0].fn*get_N(forces[0].t)[0]+forces[0].ft*get_T(forces[0].t)[0])+
              (get_P(forces[1].t)-COM)[0]*(forces[1].fn*get_N(forces[1].t)[1]+forces[1].ft*get_T(forces[1].t)[1])-
              (get_P(forces[1].t)-COM)[1]*(forces[1].fn*get_N(forces[1].t)[0]+forces[1].ft*get_T(forces[1].t)[0]);
    
    return make_pair(Ftot,Mtot);
}

/***************************************************/
bool Problem::check_no_slippage(const Force& force) const
{
    auto ft_max=friction*fabs(force.fn);
    return ((force.ft>=-ft_max) && (force.ft<=ft_max));
}

/***************************************************/
bool Problem::check_no_slippage(const vector<Force>& forces) const
{
    for (auto &f:forces) {
        if (!check_no_slippage(f)) {
            return false;
        }
    }
    return true;
}