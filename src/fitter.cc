// Copyright (C) 2012 by Antonio El Khoury.
//
// This file is part of the roboptim-capsule.
//
// roboptim-capsule is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either
// version 3 of the License, or (at your option) any later version.
//
// roboptim-capsule is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with roboptim-capsule.  If not, see
// <http://www.gnu.org/licenses/>.

/**
 * \file src/fitter.cc
 *
 * \brief Implementation of Fitter.
 */

#ifndef ROBOPTIM_CAPSULE_FITTER_CC_
# define ROBOPTIM_CAPSULE_FITTER_CC_

# include <math.h>

# include <roboptim/core/finite-difference-gradient.hh>
# include <roboptim/core/numeric-linear-function.hh>

# include "roboptim/capsule/fitter.hh"

namespace roboptim
{
  namespace capsule
  {
    // -------------------PUBLIC FUNCTIONS-----------------------

    Fitter::
    Fitter (const polyhedron_t& polyhedron) throw ()
      : polyhedron_ (polyhedron)
    {
      argument_t param (7);
      param.clear ();
      solutionParam_ = param;
    }

    Fitter::
    ~Fitter () throw ()
    {
    }

    const polyhedron_t Fitter::
    polyhedron () const throw ()
    {
      return polyhedron_;
    }

    void Fitter::
    polyhedron (const polyhedron_t& polyhedron) throw ()
    {
      assert (!!polyhedron && "Null pointer to polyhedron.");
      polyhedron_ = polyhedron;
    }

    const argument_t Fitter::
    solutionParam () const throw ()
    {
      assert (solutionParam_.size () == 7
	      && "Incorrect solutionParam size, expected 7.");

      return solutionParam_;
    }

    void Fitter::
    computeBestFitCapsule (const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedron_, initParam, solutionParam_);
    }

    void Fitter::
    computeBestFitCapsule (const polyhedron_t& polyhedron,
			   const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedron, initParam, solutionParam_);
    }

    const argument_t Fitter::
    computeBestFitCapsuleParam (const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedron_, initParam, solutionParam_);

      return solutionParam_;
    }

    const argument_t Fitter::
    computeBestFitCapsuleParam (const polyhedron_t& polyhedron,
				const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedron, initParam, solutionParam_);
      
      return solutionParam_;
    }

    // -------------------PROTECTED FUNCTIONS--------------------

    void Fitter::
    impl_computeBestFitCapsuleParam (const polyhedron_t& polyhedron,
				     const argument_t& initParam,
				     argument_t& solutionParam) throw ()
    {
      assert (!!polyhedron && "Null pointer to polyhedron");
      assert (initParam.size () == 7
	      && "Incorrect initParam size, expected 7.");
      
      // Define volume function. It is the cost of the optimization
      // problem.
      Volume volume;

      // Define optimization problem with volume as cost function.
      solver_t::problem_t problem (volume);

      // Define problem starting point.
      problem.startingPoint () = initParam;

      // The radius must not be negative.
      problem.argumentBounds ()[6] = Function::makeLowerInterval (0.);

      // Define the two distance functions. They are the constraints
      // of the optimization problem.
      DistanceSegmentPolyhedron distanceSegment (polyhedron);
      DistanceCapsulePolyhedron distanceCapsule (polyhedron);

      // Define two distance functions that implement gradient
      // computation with finite difference.
      boost::shared_ptr<FiniteDifferenceGradient<> >
	fdgDistanceSegment (new FiniteDifferenceGradient<> (distanceSegment));
      boost::shared_ptr<FiniteDifferenceGradient<> >
	fdgDistanceCapsule (new FiniteDifferenceGradient<> (distanceCapsule));

      // Add segment distance constraint. The distance between the
      // capsule segment and the polyhedron must be always positive
      // (the segment is inside the polyhedron).
      Function::interval_t distanceSegmentInterval
	= Function::makeLowerInterval (0.);

      problem.addConstraint (fdgDistanceSegment, distanceSegmentInterval, 1.);

      // Add capsule distance constraint. The distance between the
      // capsule and the polyhedron must be always negative
      // (the polyhedron is contained in the capsule).
      Function::interval_t distanceCapsuleInterval
	= Function::makeUpperInterval (0.);

      problem.addConstraint (fdgDistanceCapsule, distanceCapsuleInterval, 1.);

      std::cout << problem << std::endl;

      // Create solver using CFSQP.
      SolverFactory<solver_t> factory ("cfsqp", problem);
      solver_t& solver = factory ();
      solver.parameters ()["cfsqp.iprint"].value = 2;

      // Solve problem and check if the optimum is correct.
      solver_t::result_t result = solver.minimum ();

      switch (solver.minimumType ())
	{
	case solver_t::SOLVER_NO_SOLUTION:
	  {
	    std::cerr << "No solution." << std::endl;
	    solutionParam.clear ();
	  }
	case solver_t::SOLVER_ERROR:
	  {
	    std::cerr << "An error happened: " << std::endl
		      << solver.getMinimum<SolverError> ().what ()
		      << std::endl;
	    solutionParam.clear ();
	  }
	case solver_t::SOLVER_VALUE_WARNINGS:
	  {
	    // Display the result.
	    std::cout << "A solution has been found (minor problems occurred)"
		      << solver.getMinimum<ResultWithWarnings> ()
		      << std::endl;
	    solutionParam = solver.getMinimum<ResultWithWarnings> ().x;
	  }
	case solver_t::SOLVER_VALUE:
	  {
	    // Display the result.
	    std::cout << "A solution has been found" << std::endl
		      << solver.getMinimum<Result> () << std::endl;
	    solutionParam = solver.getMinimum<Result> ().x;
	    break;
	  }
	}
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_FITTER_CC_
