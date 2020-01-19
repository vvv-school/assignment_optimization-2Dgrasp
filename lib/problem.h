// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-
//
// Author: Ugo Pattacini - <ugo.pattacini@iit.it>

#ifndef PROBLEM_H
#define PROBLEM_H

#include <memory>
#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <cmath>
#include <algorithm>
#include <yarp/sig/Vector.h>

namespace problem_ns {

/**
 * Descriptor of the force acting on the object.
 *
 * It is given in the form F=<t,fn>, where:
 * - t:  double that identifies the location on the
 *       objet's perimeter where F is applied.
 * - fn: double that specifies the amount of force applied
 *       along the direction normal to the object's perimeter;
 *       fn>0 for inward direction. 
 * - ft: double that specifies the amount of force applied
 *       along the direction tangential to the object's perimeter;
 *       ft>0 for direction along velocity. 
 */
struct Force {
    double t{0.};
    double fn{0.};
    double ft{0.};
};

/**
 * Problem API.
 */
class Problem
{
    bool configured{false};
    std::vector<double> ti{M_PI/4.0, 3.0*M_PI/4.0, 5.0*M_PI/4.0, 7.0*M_PI/4.0};
    std::vector<double> si{.2, .2, .2, .2};
    std::vector<double> ci{0., 0., 0., 0.};
    yarp::sig::Vector COM=yarp::sig::Vector(2,0.);
    double friction{0.};
    Force F;

    size_t get_quadrant(const double t) const;
    std::tuple<double,size_t,double> calc_quantities(const double t) const;
    yarp::sig::Vector calc_COM();
    yarp::sig::Vector get_d2P(const double t) const;

public:
   /**
    * Configure the problem.
    * @param ci is a 4D vector containing the coefficients of the object's perimeter.
    * @param friction is in range [0,1].
    * @param F is the applied force.    
    * @return true/false on success/failure.
    */
    bool configure(const std::vector<double> &ci, const double friction,
                   const Force &F);

   /**
    * Generate a random problem.
    * @return the problem.
    */
    static std::shared_ptr<Problem> generate();

   /**
    * Helper function that returns the parameter t within the interval [0, 2*PI].
    * @param t is the input parameter.
    * @return the wrapped version of t.
    */
    static double wrap_angle(const double t);

   /**
    * Retrieve the current vector of coefficients describing the object's perimeter.
    * @return the perimeter's coefficients.
    */
    const std::vector<double>& get_ci() const;

   /**
    * Retrieve the friction value.
    * @return the friction.
    */
    double get_friction() const;

   /**
    * Retrieve the force applied to the object.
    * @return the force F.
    */
    const Force& get_F() const;

   /**
    * Retrieve the COM of the object.
    * @return a YARP vector containing the x and y coordinates.
    */
    const yarp::sig::Vector& get_COM() const;
        
   /**
    * Retrieve the point P on the object's perimeter.
    * @param t is the parameter.
    * @return a YARP vector containing the x and y coordinates.
    */
    yarp::sig::Vector get_P(const double t) const;
    
   /**
    * Retrieve the derivative of the point P on the object's perimeter.
    * @param t is the parameter.
    * @return a YARP vector containing the x and y coordinates.
    */
    yarp::sig::Vector get_dP(const double t) const;
    
   /**
    * Retrieve the tangent T on the object's perimeter.
    * @param t is the parameter.
    * @return a YARP vector containing the x and y coordinates.
    *
    * @note |T|≠1,  i.e. T is not a unit vector.
    * @note T·dP>0, i.e. T is aligned w/ velocity.
    */
    yarp::sig::Vector get_T(const double t) const;

   /**
    * Retrieve the derivative of the tangent T on the object's perimeter.
    * @param t is the parameter.
    * @return a YARP vector containing the x and y coordinates.
    */
    yarp::sig::Vector get_dT(const double t) const;

   /**
    * Retrieve the normal N on the object's perimeter.
    * @param t is the parameter.
    * @return a YARP vector containing the x and y coordinates.
    *
    * @note |N|≠1,     i.e. N is not a unit vector.
    * @note (T×N)·z>0, i.e. N points inward.
    */
    yarp::sig::Vector get_N(const double t) const;

   /**
    * Retrieve the derivative of the inward normal N on the object's perimeter.
    * @param t is the parameter.
    * @return a YARP vector containing the x and y coordinates.
    */
    yarp::sig::Vector get_dN(const double t) const;

   /**
    * Compute the total force and momentum of the object under the action of F and input forces.
    * @param forces is the 2D vector of the inward forces.
    * @return a pair representing the total force and momentum.
    */
    std::pair<yarp::sig::Vector,double> compute_newton_laws(const std::vector<Force>& forces) const;

   /**
    * Check if the force is within the friction cone.
    * @param force is the 2D inward force.
    * @return true if the check has passed.
    */
    bool check_no_slippage(const Force& force) const;

   /**
    * Check if the forces are within the friction cones.
    * @param forces is the vector of 2D inward forces.
    * @return true if the check has passed for all forces.
    */
    bool check_no_slippage(const std::vector<Force>& forces) const;
};

}

#endif