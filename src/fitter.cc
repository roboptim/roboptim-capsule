// Copyright (C) 2012 by Antonio El Khoury, CNRS.
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
# include <sstream>

# include <roboptim/core/finite-difference-gradient.hh>

# include <roboptim/capsule/fitter.hh>

namespace roboptim
{
  namespace capsule
  {
    // -------------------PUBLIC FUNCTIONS-----------------------

    Fitter::
    Fitter (const polyhedrons_t& polyhedrons) throw ()
      : polyhedrons_ (polyhedrons)
    {
      argument_t param (7);
      param.clear ();
      solutionParam_ = param;
    }

    Fitter::
    ~Fitter () throw ()
    {
    }

    const polyhedrons_t Fitter::
    polyhedrons () const throw ()
    {
      return polyhedrons_;
    }

    void Fitter::
    polyhedrons (const polyhedrons_t& polyhedrons) throw ()
    {
      assert (polyhedrons.size () != 0 && "Empty polyhedron vector.");
      polyhedrons_ = polyhedrons;
    }

    const value_type Fitter::
    initVolume () const throw ()
    {
      assert (initVolume_ > 0
	      && "Incorrect initial volume value. Must be non-negative.");

      return initVolume_;
    }

    const value_type Fitter::
    solutionVolume () const throw ()
    {
      assert (solutionVolume_ > 0
	      && "Incorrect solution volume value. Must be non-negative.");

      return solutionVolume_;
    }

    const argument_t Fitter::
    initParam () const throw ()
    {
      assert (initParam_.size () == 7
	      && "Incorrect initParam size, expected 7.");

      return initParam_;
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
      impl_computeBestFitCapsuleParam (polyhedrons_, initParam, solutionParam_);
    }

    void Fitter::
    computeBestFitCapsule (const polyhedrons_t& polyhedrons,
			   const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedrons, initParam, solutionParam_);
    }

    const argument_t Fitter::
    computeBestFitCapsuleParam (const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedrons_, initParam, solutionParam_);

      return solutionParam_;
    }

    const argument_t Fitter::
    computeBestFitCapsuleParam (const polyhedrons_t& polyhedrons,
				const argument_t& initParam) throw ()
    {
      impl_computeBestFitCapsuleParam (polyhedrons, initParam, solutionParam_);
      
      return solutionParam_;
    }

    // -------------------PROTECTED FUNCTIONS--------------------

    void Fitter::
    impl_computeBestFitCapsuleParam (const polyhedrons_t& polyhedrons,
				     const argument_t& initParam,
				     argument_t& solutionParam) throw ()
    {
      assert (polyhedrons.size () != 0 && "Empty polyhedron vector");
      assert (initParam.size () == 7
	      && "Incorrect initParam size, expected 7.");
      
      // Define volume function. It is the cost of the optimization
      // problem.
      Volume volume;
      initParam_ = initParam;
      initVolume_ = volume (initParam)[0];

      // Define optimization problem with volume as cost function.
      solver_t::problem_t problem (volume);

      // Define problem starting point.
      problem.startingPoint () = initParam;

      // The radius must not be negative.
      problem.argumentBounds ()[6] = Function::makeLowerInterval (0.);

      // Cycle through polyhedron points and define distance
      // functions. They are the constraints of the optimization
      // problem.
      BOOST_FOREACH (polyhedron_t polyhedron, polyhedrons)
	{
	  for (size_type j = 0; j < polyhedron.size (); ++j)
	    {
	      point_t point = polyhedron[j];
	
	      std::string s = "distance to point ";
	      std::stringstream name;
	      name << s << j;

	      // Add distance constraint. Distance must always be negative
	      // (point remains inside capsule when as it shrinks).
	      boost::shared_ptr<DistanceCapsulePoint>
		distance (new DistanceCapsulePoint (point, name.str ()));
	      Function::interval_t distanceInterval
		= Function::makeUpperInterval (0.);

	      problem.addConstraint (distance, distanceInterval, 1.);
	    }
	}

      // Create solver using Ipopt.
      SolverFactory<solver_t> factory ("ipopt", problem);
      solver_t& solver = factory ();
      solver.parameters ()["ipopt.linear_solver"].value = "mumps";
      solver.parameters ()["ipopt.derivative_test"].value = "first-order";
      solver.parameters ()["ipopt.derivative_test_perturbation"].value = 10e-8;
      solver.parameters ()["ipopt.print_level"].value = 5;
      solver.parameters ()["ipopt.file_print_level"].value = 5;
      solver.parameters ()["ipopt.print_user_options"].value = "yes";
      solver.parameters ()["ipopt.output_file"].value = "fitter-ipopt.out";

      // Solve problem and check if the optimum is correct.
      solver_t::result_t result = solver.minimum ();

      switch (solver.minimumType ())
	{
	case solver_t::SOLVER_NO_SOLUTION:
	  {
	    std::cerr << "No solution." << std::endl;
	    solutionParam = initParam_;
	    break;
	  }
	case solver_t::SOLVER_ERROR:
	  {
	    // Display error and fall back gracefully to initial
	    // guess.
	    std::cerr << "An error happened: " << std::endl
	    	      << solver.getMinimum<SolverError> ().what ()
	    	      << std::endl;
	    solutionParam = initParam_;
	    break;
	  }
	case solver_t::SOLVER_VALUE_WARNINGS:
	  {
	    // Display the result.
	    std::cout << "A solution has been found (minor problems occurred)"
		      << solver.getMinimum<ResultWithWarnings> ()
		      << std::endl;
	    solutionParam = solver.getMinimum<ResultWithWarnings> ().x;
	    break;
	  }
	case solver_t::SOLVER_VALUE:
	  {
	    // Display the result.
	    std::cout << "A solution has been found" << std::endl;
	    solutionParam = solver.getMinimum<Result> ().x;
	    break;
	  }
	}

      solutionParam_ = solutionParam;
      solutionVolume_ = volume (solutionParam)[0];
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_FITTER_CC_
