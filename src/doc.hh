// Copyright (C) 2012 by Antonio El Khoury, CNRS.
//
// This file is part of the roboptim-capsule.
//
// roboptim-capsule is free software: you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public License
// as published by the Free Software Foundation, either version 3 of
// the License, or (at your option) any later version.
//
// roboptim-capsule is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with roboptim-capsule.  If not, see
// <http://www.gnu.org/licenses/>.

/**

   \mainpage User manual

   \section intro Introduction

   roboptim-capsule defines a C++ implementation of a minimum-volume
   capsule optimizer over a polyhedron or vector of polyhedrons.

   A capsule is a geometric surface defined as the set of points lying
   at a fixed distance from a segment. It is a convex surface that can
   be very useful to give a simplified representation of robot bodies
   in motion planning and optimal control. However, it can be a
   non-trivial task to compute an "optimal" capsule that approximates
   best the underlying geometry, which is usually modeled by one or
   more polyhedrons.

   A capsule can be uniquely defined by its two main axis end points
   \f$e_1\f$ and \f$e_2\f$, as well as its radius \f$r\f$. The
   roboptim-capsule optimizer takes as input one or more polyhedrons,
   and computes the minimum-volume capsule parameters, i.e. \f$e_1\f$,
   \f$e_2\f$, and \f$r\f$.

   The package relies on:
   <ul>
   <li>geometric types and operations (<a href="http://www.geometrictools.com/">geometric-tools</a>, aka WildMagic),</li>
   <li>and numerical optimization (<a href="http://roboptim.net/">RobOptim and its IPOPT plugin</a>).</li>
   </ul>

   To get basic knowledge about the library, you might want to check out the
   \ref quickstart page.

   \section reporting Reporting bugs

   As this package is still in its early development steps, bugs report
   and enhancement proposals are highly welcomed.

   To report a bug, please go this package's
   <a href="https://github.com/laas/roboptim-capsule/">web page</a>.
*/

/**
   \page quickstart Quick start

   \section problem Problem definition

   A capsule can be uniquely defined by its two main axis end points
   \f$e_1\f$ and \f$e_2\f$, as well as its radius \f$r\f$. To compute
   the minimum-volume bounding capsule over one or more polyhedrons,
   an non-linear optimization problem is defined, then solved using a
   jacobian-based numerical optimization solver provided through
   RobOptim and an IPOPT plugin.

   The problem that will be solved is:

   \f$min_{e_1, e_2, r} \|e_2 - e_1\| \pi r^2 + \frac{4}{3}\pi r^3\f$

   subject to:

   \f$r - d(p,e_1e_2) \geq p \in \mathcal{P}\f$

   This ensures that the minimum-volume capsule is found while all
   points of the set \f$\mathcal{P}\f$, which represents one or more
   polyhedrons, all lie inside the capsule.

   \section problem Building the problem and solving it.

   This part of the tutorial covers how to compute an optimal
   capsule. The steps are:

   - Instanciate or load a polyhedron. Here we create a simple cube.

   \code
   BOOST_AUTO_TEST_CASE (fitter)
   {
   using namespace roboptim::capsule;
   
   // Build a cubic polyhedron centered in (0,0,0).
   polyhedron_t polyhedron;
   value_type halfLength = 0.5;

   polyhedron.push_back (point_t (-halfLength, -halfLength, -halfLength));
   polyhedron.push_back (point_t (-halfLength, -halfLength, halfLength));
   polyhedron.push_back (point_t (-halfLength, halfLength, -halfLength));
   polyhedron.push_back (point_t (-halfLength, halfLength, halfLength));
   polyhedron.push_back (point_t (halfLength, -halfLength, -halfLength));
   polyhedron.push_back (point_t (halfLength, -halfLength, halfLength));
   polyhedron.push_back (point_t (halfLength, halfLength, -halfLength));
   polyhedron.push_back (point_t (halfLength, halfLength, halfLength));

   polyhedrons_t polyhedrons;
   polyhedrons.push_back (polyhedron);
   \endcode

   - Compute the convex hull of one or more polyhedrons. This offers
     the nice property that a lower number of constraints will be
     added to the optimization problem without affecting the result.

   \code
   polyhedrons_t convexPolyhedrons;
   computeConvexPolyhedron (polyhedrons, convexPolyhedrons);
   \endcode

   - Instanciate a fitter. This is the main class in this package and
     is used to find the best fitting capsule over the input
     polyhedron vector.
   
   \code
   Fitter fitter (convexPolyhedrons);
   \endcode

   - Set initial parameters for the fitter. One very nice way to find
     a good initial guess is to compute a bounding capsule with
     least-squares method. This way, the initial capsule enforces all
     constraints and is not too far from the optimum.

   \code
   point_t endPoint1, endPoint2;
   value_type radius;
   computeBoundingCapsulePolyhedron (convexPolyhedrons,
   endPoint1, endPoint2, radius);

   argument_t initParam (7);
   convertCapsuleToSolverParam (initParam, endPoint1, endPoint2, radius);
   \endcode

   - Run the solver and retrieve the optimal capsule parameters.

   \code
   fitter.computeBestFitCapsule (initParam);
   argument_t solutionParam = fitter.solutionParam ();
   std::cout << fitter << std::endl;
   }
   \endcode

   To see more usage examples, consider looking at the test directory of the
   project which contains the project's test suite.
*/
