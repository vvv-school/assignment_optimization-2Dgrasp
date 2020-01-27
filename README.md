Nonlinear Constrained Optimization with Ipopt
=============================================

[![Gitpod](https://gitpod.io/button/open-in-gitpod.svg)](https://gitpod.io/#https://github.com/vvv-school/assignment_optimization-2Dgrasp)

# Prerequisites
To tackle this assignment you would need to know the following:
- How to deal with simple [Linear Algebra](https://en.wikipedia.org/wiki/Linear_algebra).
- A tiny bit of Physics. In particular, the concepts of [Force](https://en.wikipedia.org/wiki/Force) and [Torque](https://en.wikipedia.org/wiki/Torque).
- How to apply [Newton's First Law](https://en.wikipedia.org/wiki/Newton%27s_laws_of_motion#Newton's_first_law) for linear and rotational motion.
- A bare understanding of [static friction](https://en.wikipedia.org/wiki/Friction).
- The basics of [Ipopt](https://coin-or.github.io/Ipopt).

# Assignment
You are set in a **2D world** where an object is given with an irregular shape (**Figure 1**). The object is characterized by a **static friction coefficient**
and there exists already a force `F0` applied to a certain location of the object's perimeter, pointing inward. As it is known, The `friction cone` establishes
an area − thinner for smaller coefficients and wider for larger coefficients − where `F0` does not bring about any slippage between the object and the effector
of an external agent exerting the force. 

| Figure 1 |
| :---: |
| ![object](/assets/object.png) |

🎯 The goal is to **find out a suitable pair of forces** `F1` and `F2` that when applied to the object together with `F0` are capable of producing a **stable grasp**.
To achieve this, the forces need to **point inward** and guarantee that **Newton's first law** is satisfied both for linear and rotational motion.
Further, to **prevent slippage**, `F1` and `F2` are requested to lie within their **friction cones**.

👉 Think of `F0`, `F1` and `F2` as the solution of a 2D planar grasp problem with a 3 fingers gripper, where the force applied by one finger (i.e. `F0`) is already determined.

## Problem Settings
The object's perimeter is contained in the x-y plane and is parmetrized in terms of the angular position `t` specified in radians. Moreover, the [Problem API](https://vvv-school.github.io/assignment_optimization-2Dgrasp/doxygen/doc/html/classproblem__ns_1_1Problem.html) provides you with the suitable routines to retrieve the point `P`, the normal `N` and the tangent `T` as function of the parameter `t` (**Figure 2**). Also, the API gives you the derivatives of such quantities with respect to `t`. 

⚠ Be careful that `N(t)` and `T(t)` are **NOT unit vectors**.

| Figure 2 |
| :---: |
| ![settings](/assets/settings.png) |

In particular, with these settings, a [Force](https://vvv-school.github.io/assignment_optimization-2Dgrasp/doxygen/doc/html/structproblem__ns_1_1Force.html) can be described by the triplet <`t`,`fn`,`ft`>, where:
- `t` accounts for the angular position where the force is applied.
- `fn` specifies the amount of force applied along `N(t)`.
- `ft` specifies the amount of force applied along `T(t)`.

## Physics in the 2D World
The assumption of a 2D world helps simplify the Problem settings such that a force boils down to a **2D vector in the x-y plane**, whereas a torque can be represented
with only a **scalar along the z axis**. In detail, we can take advantage of the class [`yarp::sig::Vector`](http://yarp.it/classyarp_1_1sig_1_1VectorOf.html) and
the utilities defined in [`yarp/math/Math.h`](http://yarp.it/Math_8h.html) to deal with the required linear algebra (e.g. summation of vectors, inner and outer products).

The net force `Ftot` and net torque `Ttot` can be readily obtained by recruiting Netwon's law:
```c++
// F0 relative quantities
problem_ns::Force F0=problem.get_F();
yarp::sig::Vector P0=problem.get_P(F0.t);
yarp::sig::Vector N0=problem.get_N(F0.t);
yarp::sig::Vector T0=problem.get_T(F0.t);

// F1 and F2 are to be provided
problem_ns::Force F1, F2;
// retrieve P1,N1,T1 and P2,N2,T2 analogously

// linear (x,y axes)
yarp::sig::Vector Ftot=F0.fn*N0+F1.fn*N1+F2.fn*N2+F0.ft*T0+F1.ft*T1+F2.ft*T2;

// Center Of Mass of the object
yarp::sig::Vector COM=problem.get_COM();

// rotation (z-axis): only the third component of the outer product is relevant
double Ttot=(P0-COM)[0]*(F0.fn*N0[1]+F0.ft*T0[1])-(P0-COM)[1]*(F0.fn*N0[0]+F0.ft*T0[0])+
            (P1-COM)[0]*(F1.fn*N1[1]+F1.ft*T1[1])-(P1-COM)[1]*(F1.fn*N1[0]+F1.ft*T1[0])+
            (P2-COM)[0]*(F2.fn*N2[1]+F2.ft*T2[1])-(P2-COM)[1]*(F2.fn*N2[0]+F2.ft*T2[0]);
```

## Code Structure
The relevant code structure is reported below:
```sh
assignment_optimization-2Dgrasp
├── CMakeLists.txt                  # To build the library and the main code
├── lib                             # Code library
│   ├── problem.h                   # Definition of Problem API
│   ├── problem.cpp                 # Problem implementation
│   ├── solver.h                    # Definition of Solver API
│   └── solver.cpp                  # Your implementation of the Solver (YOU HAVE TO WORK OUT THE CONTENT OF THIS FILE)
├── scripts
│   └── plot_2Dgrasp-problem.m      # Generate a graphical representation of the Problem settings along with your solution
├── smoke-test
│   ├── test.sh                     # Run the complete test suite 
└── src
    └── main.cpp                    # Main code to test your implementation
```

You are asked to develop within the file `lib/solver.cpp` the solution that exploits the nonlinear constrained optimization package Ipopt.

The library is [documented online](https://vvv-school.github.io/assignment_optimization-2Dgrasp) 🌐

### Build the Code
```sh
$ cd assignment_optimization-2Dgrasp
$ mkdir -p build && cd build
$ cmake ..
$ make install
```

### Test the Code
To test your solution against an object whose perimeter is a perfect circle do:
```sh
$ assignment_optimization-2Dgrasp --shape circle
```

To test your solution against an object whose perimeter is an irregular patch do:
```sh
$ assignment_optimization-2Dgrasp --shape patch
```

In both cases, the outcome can be conveniently displayed this way:
```sh
$ plot_2Dgrasp-problem problem.out
```
Then, open up the file `problem.out.png`. Figure 3 illustrates a typical outcome.

| Figure 3 |
| :---: |
| ![example-solution](/assets/example-solution.png) |

Once you deem you're ready to go, you can accept the challenge of the grading test suite by doing:
```sh
$ cd assignment_optimization-2Dgrasp/smoke-test
$ ./test.sh
```

<details>
<summary>🔘 Click to show how the grading system will assign you score</summary>

---
The test suite will perform two consecutive verifications:
1. A Problem with an object whose shape is a **perfect cirlce** is generated **100 times** and checks are done to verify the grasp stability of your solution.
   The force `F0` is always set **normal to the perimeter**.
1. A Problem with an object whose shape is a **irregular patch** is generated **100 times** and checks are done to verify the grasp stability of your solution.
   The force `F0` can be generically **oriented inward within its friction cone**.

The score is then computed statistically over the 100 trials according to the following requirements.

#### R1. Requirements to satisfy with a circle-shaped object
1. **Linear stability**. The net force F shall be in norm smaller than 0.01: 100% of success rate amounts to 4 points.
1. **Rotational stability**. The torque T shall be in norm smaller than 0.01: 100% of success rate amounts to 4 points.
1. **No slippage**. The two forces provided by your algorithm shall be contained within the friction cones to prevent slippage: 100% of success rate amounts to 4 points.

#### R2. Requirements to satisfy with a patch-shaped object
1. **Linear stability**. The net force F shall be in norm smaller than 0.01: 100% of success rate amounts to 4 points.
1. **Rotational stability**. The net torque T shall be in norm smaller than 0.01: 100% of success rate amounts to 4 points.
1. **No slippage**. The two forces provided by your algorithm shall be contained within the friction cones to prevet slippage: 100% of success rate amounts to 4 points.

#### 🌟 Bonus
If **R2.* ≥ 98%**, then you will get the **corresponding points doubled**.

#### Score Map
| Requirements | Points |
|:---:|:---:|
| R1.1 | 0 … 4 |
| R1.2 | 0 … 4 |
| R1.3 | 0 … 4 |
| R2.1 | 0 … 8 |
| R2.2 | 0 … 8 |
| R2.3 | 0 … 8 |

The maximum score you can achieve is therefore **36** 🏆

---
</details>

## Incremental Approach
We recommend that you tackle the assignment incrementally:
1. First, [**peruse the solution we provided**](https://github.com/vvv-school/assignment_optimization-2Dgrasp/wiki/Solution-for-Linear-Motion-Only) for the case where we handle only normal forces and we neglect torques. Therefore, grasp stability can be attained only for **linear motion**.
1. Then, extend the solution in order to incorporate Newton's first law also for **rotational motion**.
1. Finally, consider friction cones and thus deal with **tangential components** of the forces, while avoiding slippage.

# [How to complete the assignment](https://github.com/vvv-school/vvv-school.github.io/blob/master/instructions/how-to-complete-assignments.md)