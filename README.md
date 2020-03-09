# Control Toolbox
![GitHub top language](https://img.shields.io/github/languages/top/ethz-adrl/control-toolbox.svg?style=social)
![GitHub](https://img.shields.io/github/license/ethz-adrl/control-toolbox.svg?style=social)

This is the Control Toolbox, an efficient C++ library for control, estimation, optimization and motion planning in robotics.

Link to the wiki, [quickstart!](https://github.com/ethz-adrl/control-toolbox/wiki/Quickstart) \
Find the detailed documentation [here](https://ethz-adrl.github.io/ct/).


## Overview

This is the ADRL Control Toolbox ('CT'), an open-source C++ library for efficient modelling, control, 
estimation, trajectory optimization and model predictive control. The CT is applicable to a broad class of dynamic systems, but features additional modelling tools specially designed for robotics. This page outlines its general concept, its major building blocks and highlights selected application examples.

The library contains several tools to design and evaluate controllers, model dynamical systems and solve optimal control problems.
The CT was designed with the following features in mind:

 - **Systems and dynamics**: 
	- intuitive modelling of systems governed by ordinary differential or difference equations.

 - **Trajectory optimization, optimal control and (nonlinear) model predictive control**:
    - intuitive modelling of cost functions and constraints
    - common interfaces for optimal control solvers and nonlinear model predictive control
    - currently supported algorithms:
      - Classical Single Shooting
      - iLQR / iLQG (iterative Linear Quadratic Optimal Control)
      - Multiple-shooting iLQR
      - Gauss-Newton-Multiple-Shooting (GNMS)
      - Classical Direct Multiple Shooting (DMS)
    - standardized interfaces for the solvers
      - IPOPT (first and second order)
      - SNOPT
      - HPIPM
      - custom Riccati-solver
 
 - **Performance**: 
 	- solve large scale optimal control problems in MPC fashion. 
  
 - **Robot Modelling, Rigid Body Kinematics and Dynamics**: 
    - straight-forward interface to the state-of the art rigid body dynamics modelling tool RobCoGen.
	- implementation of a basic nonlinear-programming inverse kinematics solver for fix-base robots.
 
 - **Automatic Differentiation**:
    - first and second order automatic differentiation of arbitrary vector-valued functions including cost functions and constraints
    - automatic differentiation and code generation of rigid body dynamics
    - derivative code generation for maximum efficiency


## Robot Application Examples

The Control Toolbox has been used for Hardware and Simulation control tasks on flying, walking and ground robots.

<a href="http://www.youtube.com/watch?feature=player_embedded&v=Y7-1CBqs4x4" target="_blank"><img src="http://img.youtube.com/vi/Y7-1CBqs4x4/0.jpg" 
alt="Control Toolbox example video" width="360" height="270" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=vuCSKtP67E4" target="_blank"><img src="http://img.youtube.com/vi/vuCSKtP67E4/0.jpg" 
alt="Control Toolbox example video" width="360" height="270" border="10" /></a>

<a href="http://www.youtube.com/watch?feature=player_embedded&v=rWmw-ERGyz4" target="_blank"><img src="http://img.youtube.com/vi/rWmw-ERGyz4/0.jpg" 
alt="Control Toolbox example video" width="360" height="270" border="10" /></a>
<a href="http://www.youtube.com/watch?feature=player_embedded&v=rVu1L_tPCoM" target="_blank"><img src="http://img.youtube.com/vi/rVu1L_tPCoM/0.jpg" 
alt="Control Toolbox example video" width="360" height="270" border="10" /></a>

Slightly more complex optimization examples, including gait optimization for a quadruped, are availabe in [ct_ros](https://github.com/ethz-adrl/ct_ros).


## What is the CT?

A common tasks for researchers and practitioners in both the control and the robotics communities 
is to model systems, implement equations of motion and design model-based controllers, estimators, 
planning algorithms, etc.
Sooner or later, one is confronted with questions of efficient implementation, computing derivative 
information, formulating cost functions and constraints or running controllers in model-predictive 
control fashion.

The Control Toolbox is specifically designed for these tasks. It is written entirely in C++ and has 
a strong focus on highly efficient code that can be run online (in the loop) on robots or other 
actuated hardware. A major contribution of the CT is its implementations of optimal control 
algorithms, spanning a range from simple LQR reference implementations to constrained model predictive 
control. The CT supports Automatic Differentiation (Auto-Diff) and allows to generate derivative code 
for arbitrary scalar and vector-valued functions. 
We designed the toolbox with usability in mind, allowing users to apply advanced concepts such as 
nonlinear model predictive control (NMPC) or numerical optimal control easily and with minimal effort.
While we provide an interface to a state-of-the art Auto-Diff compatible robot modelling software, all 
other modules are independent of the a particular modelling framework, allowing the code to be interfaced 
with existing C/C++ code or libraries. 

The CT has been successfully used in a variety of different projects, including a large number of 
hardware experiments, demonstrations and academic publications. 
Example hardware applications are online trajectory optimization with collision 
avoidance \cite giftthaler2017autodiff, trajectory optimization for Quadrupeds \cite neunert:2017:ral 
and mobile manipulators \cite giftthaler2017efficient as well as NMPC on ground robots \cite neunert2017mpc 
and UAVs \cite neunert16hexrotor. The project originated from research conducted at the Agile \& Dexterous 
Robotics Lab at ETH Zurich, but is continuously extended to cover more fields of applications and algorithms.
 

## Scope of the CT

 Software is one of the key building blocks for robotic systems and there is a great effort 
 in creating software tools and libraries for robotics. 
 However, when it comes to control and especially Numerical Optimal Control, there are not 
 many open source tools available that are both easy to use for fast development as well as 
 efficient enough for online usage. 
 While there exist mature toolboxes for Numerical Optimal Control and Trajectory Optimization, 
 they are highly specialized, standalone tools that due not provide sufficient flexibility 
 for other applications. Here is where the CT steps in. The CT has been designed from ground 
 up to provide the tools needed for fast development and evaluation of control methods while 
 being optimized for efficiency allowing for online operation. While the emphasis lies on control, 
 the tools provided can also be used for simulation, estimation or optimization applications.

 In contrast to other robotic software, CT is not a rigid integrated application but can be 
 seen quite literal as a toolbox: It offers a variety of tools which can be used and combined 
 to solve a task at hand. While ease-of-use has been a major criteria during the design and 
 application examples are provided, using CT still requires programming and control knowledge. 
 However, it frees the users from implementing standard methods that require in-depth experience 
 with linear algebra or numerical methods. Furthermore, by using common definitions and types, 
 a seamless integration between different components such as systems, controllers or integrators 
 is provided, enabling fast prototyping.


## Design and Implementation

The main focus of CT is efficiency, which is why it is fully implemented in C++. 
Since CT is designed as a toolbox rather than an integrated application, we tried to provide 
maximum flexibility to the users. Therefore, it is not tied to a specific middleware such as 
ROS and dependencies are kept at a minimum. The two essential dependencies for CT are
<a href="http://eigen.tuxfamily.org" target="_blank">Eigen</a> and 
<a href="http://github.com/ethz-asl/kindr" target="_blank">Kindr</a>
(which is based on Eigen). This Eigen dependency is intentional since Eigen
is a defacto standard for linear algebra in C++, as it provides highly efficient implementations 
of standard matrix operations as well as more advanced linear algebra methods. 
Kindr is a header only Kinematics library which builds on top of it and provides data types 
for different rotation representations such as Quaternions, Euler angles or rotation matrices.


## Structure and Modules of the CT

The Control Toolbox consists of three main modules. The Core (`ct_core`) module, the Optimal 
Control (`ct_optcon`) module and the Rigid Body Dynamics (`ct_rbd`) module. 
There is a clear hierarchy between the modules. 
That means, the modules depend on each other in this order, e.g. you can use the core module 
without the optcon or rbd module.
 - The Core (`ct_core`) module provides general type definitions and mathematical tools.
For example, it contains most data type definitions, definitions for systems and controllers, 
as well as basic functionality such as numerical integrators for differential equations. 
 - The Optimal Control (`ct_optcon`) module builds on top of the Core module and adds 
infrastructure for defining and solving Optimal Control Problems. It contains the functionality
for defining cost functions, constraints, solver backends and a general MPC wrapper.
 - The Rigid Body Dynamics (`ct_rbd`) module provides tools for modelling Rigid Body Dynamics 
systems and interfaces with `ct_core` and `ct_optcon` data types. 

For testing as well as examples, we also provide the models module (`ct_models`) which contains 
various robot models including a quadruped, a robotic arm, a normal quadrotor and a quadrotor 
with slung load. 

The four different main modules are detailed in the following.


### ct_core (Core)

 - Definitions of fundamental types for **control** and simulation, such as dynamic systems 
 (ct::core::System), states (ct::core::StateVector), controls (ct::core::Controller), or trajectories 
 (ct::core::DiscreteTrajectoryBase).
 - **Numeric integration** (ct::core::Integrator) with various ODE solvers including fixed step 
 (ct::core::IntegratorEuler, ct::core::IntegratorRK4) and variable step (ct::core::IntegratorRK5Variable, 
 ct::core::ODE45) integrators, as well as symplectic (semi-implicit) integrators.
 - Numerical approximation of Trajectory **Sensitivities** (ct::core::Sensitivity , e.g. by forward-integrating 
 variational differential equations)
 - Common **feedback controllers** (e.g. ct::core::PIDController)
 - Derivatives/Jacobians of general functions using **Numerical Differentiation** (ct::core::DerivativesNumDiff) 
 or **Automatic-Differentiation** with code-generation (ct::core::DerivativesCppadCG) and just-in-time (JIT) 
 compilation (ct::core::DerivativesCppadJIT)
 

### ct_optcon (Optimal Control)

 - Definitions for **Optimal Control Problems** (ct::optcon::OptConProblem) and **Optimal Control Solvers** (ct::optcon::OptConSolver)
 - **CostFunction toolbox** allowing to construct cost functions from file and providing first-order and **second-order approximations**, see ct::optcon::CostFunction.
 - **Constraint toolbox** for formulating constraints of Optimal Control Problems, as well as automatically computing their Jacobians.
 - reference C++ implementations of the **Linear Quadratic Regulator**, infinite-horizon LQR and time-varying LQR (ct::optcon::LQR, ct::optcon::TVLQR)
 - **Riccati-solver** (ct::optcon::GNRiccatiSolver) for unconstrained linear-quadratic optimal control problems, interface to high-performance 
 <a href="https://github.com/giaf/hpipm" target="_blank"> third-party Riccati-solvers</a> for constrained linear-quadratic optimal control problems
 - **iterative non-linear Optimal Control** solvers, i.e. Gauss-Newton solvers such as iLQR (ct::optcon::iLQR) and 
 Gauss-Newton Multiple Shooting(ct::optcon::GNMS), constrained direct multiple-shooting (ct::optcon::DmsSolver)
 - Non-Linear **Model Predictive Control** (ct::optcon::MPC)
 - Definitions for Nonlinear Programming Problems (**NLPs**, ct::optcon::Nlp) and interfaces to third-party **NLP Solvers** 
 (ct::optcon::SnoptSolver and ct::optcon::IpoptSolver)
  
  
### ct_rbd (Rigid Body Dynamics)

 - Standard models for Rigid Body Dynamics
 - Definitions for the state of a Rigid Body System expressed as general coordinates (ct::rbd::RBDState)
 - Routines for different flavors of **Forward** and **Inverse Dynamics** (ct::rbd::Dynamics)
 - Rigid body and end-effector **kinematics** (ct::rbd::Kinematics)
 - Operational Space Controllers
 - Basic soft **auto-differentiable contact model** for arbitrary frames (ct::rbd::EEContactModel)
 - **Actuator dynamics** (ct::rbd::ActuatorDynamics)
 - Backend uses <a href="https://bitbucket.org/mfrigerio17/roboticscodegenerator/" target="_blank">RobCoGen</a> \cite frigerioCodeGen, 
 a highly efficient Rigid Body Dynamics library


### ct_models

 - Various standard models for testing and evaluation including UAVs (ct::models::Quadrotor), ground robots, 
 legged robots (ct::models::HyQ), robot arms (ct::models::HyA), inverted pendulums etc.
 - Means of creating linear approximation of these models
 

## How to get started

To get started with the control toolbox, please build the repository documentation with doxygen and follow the "Getting Started" tutorial.


## Support
- contact the devs: control-toolbox-dev@googlegroups.com


## Acknowledgements

### Contributors
 - Markus Giftthaler, markusgft (at) gmail (dot) com
 - Michael Neunert
 - Markus Stäuble
 - Farbod Farshidian
 - Diego Pardo
 - Timothy Sandy
 - Jan Carius
 - Ruben Grandia
 - Hamza Merzic
 
### Maintenance
 - Markus Giftthaler, markusgft (at) gmail (dot) com


### Funding
This software has been developed at the <a href="http://www.adrl.ethz.ch" target="_blank">Agile & Dexterous Robotics Lab</a> 
at <a href="http://www.ethz.ch/en" target="_blank">ETH Zurich</a>, Switzerland between 2014 and 2018.
During that time, development has been made possible through financial support from the <a href="http://www.snf.ch/en/" target="_blank">Swiss National Science Foundation (SNF)</a> 
through a SNF Professorship award to Jonas Buchli and the National Competence Centers in Research (NCCR) 
<a href="https://www.nccr-robotics.ch/" target="_blank">Robotics</a> and <a href="http://www.dfab.ch/en/" target="_blank">Digital Fabrication</a>.


## Licence Information

The Control Toolbox is released under the 
<a href="https://choosealicense.com/licenses/bsd-2-clause/" target="_blank">BSD-2 clause license</a>.
Please see LICENCE.txt and NOTICE.txt


##  How to cite the CT

    @article{adrlCT,
      title={The control toolbox — An open-source C++ library for robotics, optimal and model predictive control},
      author={Markus Giftthaler and Michael Neunert and Markus St{\"a}uble and Jonas Buchli},
      journal={2018 IEEE International Conference on Simulation, Modeling, and Programming for Autonomous Robots (SIMPAR)},
      year={2018},
      pages={123-129}
    }

## Earlier Versions

Earlier versions up to v2.3 are hosted on bitbucket, they can be found at https://bitbucket.org/adrlab/ct/wiki/Home


## Related Publications

This toolbox has been used in, or has helped to realize the following academic publications:

- Markus Giftthaler, Michael Neunert, Markus Stäuble and Jonas Buchli.
“The Control Toolbox - An Open-Source C++ Library for Robotics, Optimal and Model Predictive Control”. 
IEEE Simpar 2018 (Best Student Paper Award).
<a href="https://arxiv.org/abs/1801.04290" target="_blank">arXiv preprint</a>

- Markus Giftthaler, Michael Neunert, Markus Stäuble, Jonas Buchli and Moritz Diehl.
“A Family of iterative Gauss-Newton Shooting Methods for Nonlinear Optimal Control”.
IROS 2018. 
<a href="https://arxiv.org/abs/1711.11006" target="_blank">arXiv preprint</a>

- Jan Carius, René Ranftl, Vladlen Koltun and Marco Hutter. 
"Trajectory Optimization with Implicit Hard Contacts." 
IEEE Robotics and Automation Letters 3, no. 4 (2018): 3316-3323.

- Michael Neunert, Markus Stäuble, Markus Giftthaler, Dario Bellicoso, Jan Carius, Christian Gehring, 
Marco Hutter and Jonas Buchli. 
“Whole Body Model Predictive Control Through Contacts For Quadrupeds”.
IEEE Robotics and Automation Letters, 2017.
<a href="https://arxiv.org/abs/1712.02889" target="_blank">arXiv preprint</a>

- Markus Giftthaler and Jonas Buchli.
“A Projection Approach to Equality Constrained Iterative Linear Quadratic Optimal Control”. 
2017 IEEE-RAS International Conference on Humanoid Robots, November 15-17, Birmingham, UK.
<a href="http://ieeexplore.ieee.org/document/8239538/" target="_blank">IEEE Xplore</a>

- Markus Giftthaler, Michael Neunert, Markus Stäuble, Marco Frigerio, Claudio Semini and Jonas Buchli. 
“Automatic Differentiation of Rigid Body Dynamics for Optimal Control and Estimation”, 
Advanced Robotics, SIMPAR special issue. November 2017. 
<a href="https://arxiv.org/abs/1709.03799" target="_blank">arXiv preprint</a> 

- Michael Neunert, Markus Giftthaler, Marco Frigerio, Claudio Semini and Jonas Buchli. 
“Fast Derivatives of Rigid Body Dynamics for Control, Optimization and Estimation”, 
2016 IEEE International Conference on Simulation, Modelling, and Programming for Autonomous Robots, 
San Francisco. (Best Paper Award). 
<a href="http://ieeexplore.ieee.org/document/7862380/" target="_blank">IEEE Xplore</a> 

- Michael Neunert, Farbod Farshidian, Alexander W. Winkler, Jonas Buchli
"Trajectory Optimization Through Contacts and Automatic Gait Discovery for Quadrupeds",
IEEE Robotics and Automation Letters,
<a href="http://ieeexplore.ieee.org/document/7845678/" target="_blank">IEEE Xplore</a>

- Michael Neunert, Cédric de Crousaz, Fadri Furrer, Mina Kamel, Farbod Farshidian, Roland Siegwart, Jonas Buchli.
"Fast nonlinear model predictive control for unified trajectory optimization and tracking",
2016 IEEE International Conference on Robotics and Automation (ICRA),
<a href="http://ieeexplore.ieee.org/document/7487274/" target="_blank">IEEE Xplore</a> 

- Markus Giftthaler, Farbod Farshidian, Timothy Sandy, Lukas Stadelmann and Jonas Buchli. 
“Efficient Kinematic Planning for Mobile Manipulators with Non-holonomic Constraints Using Optimal Control”, 
IEEE International Conference on Robotics and Automation, 2017, Singapore.
<a href="https://arxiv.org/abs/1701.08051" target="_blank">arXiv preprint</a> 

- Markus Giftthaler, Timothy Sandy, Kathrin Dörfler, Ian Brooks, Mark Buckingham, Gonzalo Rey, 
Matthias Kohler, Fabio Gramazio and Jonas Buchli. 
“Mobile Robotic Fabrication at 1:1 scale: the In situ Fabricator”. Construction Robotics, Springer Journal no. 41693
<a href="https://arxiv.org/abs/1701.03573" target="_blank">arXiv preprint</a> 

- Timothy Sandy, Markus Giftthaler, Kathrin Dörfler, Matthias Kohler and Jonas Buchli.
“Autonomous Repositioning and Localization of an In situ Fabricator”, 
IEEE International Conference on Robotics and Automation 2016, Stockholm, Sweden.
<a href="http://ieeexplore.ieee.org/document/7487449/" target="_blank">IEEE Xplore</a> 

- Michael Neunert, Farbod Farshidian, Jonas Buchli (2014). Adaptive Real-time Nonlinear Model Predictive Motion Control. 
In IROS 2014 Workshop on Machine Learning in Planning and Control of Robot Motion
<a href="http://www.adrl.ethz.ch/archive/p_14_mlpc_mpc_rezero.pdf" target="_blank">preprint</a> 
