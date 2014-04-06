// Copyright (C) 2014 by Benjamin Chrétien, CNRS-LIRMM.
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
 * \file src/capsule-generator.cc
 *
 * \brief CLI capsule generator.
 */

#include <iostream>
#include <string>

#include <boost/program_options.hpp>

#include <roboptim/capsule/fitter.hh>
#include <roboptim/capsule/util.hh>

using namespace roboptim;
using namespace roboptim::capsule;

int main(int argc, char** argv)
{
  try
    {
      namespace po = boost::program_options;

      po::options_description desc ("Options");
      desc.add_options ()
	("help", "Print this help and exit")
	("solver", po::value<std::string> (), "Nonlinear solver used")
	("points", po::value<std::vector<double> > ()->multitoken ()->required (),
	 "Points that will be encapsulated");

      po::positional_options_description positionalOptions;

      po::variables_map vm;

      try
	{
	  po::store (po::command_line_parser (argc, argv)
		     .options (desc)
		     .positional (positionalOptions)
		     .style(
			    po::command_line_style::unix_style
			    ^ po::command_line_style::allow_short
			    )
		     .run (),
		     vm);

	  // Display help message
	  if (vm.count ("help"))
	    {
	      std::cout << desc;
	      return EXIT_SUCCESS;
	    }

	  std::string solver = "ipopt";

	  // Load (optional) NLP solver
	  if (vm.count ("solver"))
	    {
	      solver = vm["solver"].as<std::string> ();
	    }

	  // Check that points data was given
	  if (!vm.count ("points"))
	    {
	      std::cerr << "Error: missing mandatory point data." << std::endl;
	      return EXIT_FAILURE;
	    }

	  // Get points from CLI options
	  const std::vector<double>&
	    points = vm["points"].as<std::vector<double> > ();

	  if (points.size ()%3 != 0)
	    {
	      std::cerr << "Error: points should be an array of 3D points, "
			<< "e.g. x0 y0 z0 x1 y1 z1 etc." << std::endl;
	      return EXIT_FAILURE;
	    }

	  // Load polyhedron
	  polyhedron_t polyhedron;
	  for (size_t i = 0; i < points.size (); i+=3)
	    {
	      point_t p (points[i], points[i+1], points[i+2]);
	      polyhedron.push_back (p);
	    }

	  // Fitter expects a vector of polyhedrons
	  polyhedrons_t polyhedrons;
	  polyhedrons.push_back (polyhedron);

	  // Create fitter
	  Fitter fitter (polyhedrons, solver);

	  // Compute initial guess
	  point_t P0;
	  point_t P1;
	  value_type r = 0.;
	  argument_t initParam (7);

	  polyhedrons_t convexPolyhedrons;
	  computeConvexPolyhedron (polyhedrons, convexPolyhedrons);
	  computeBoundingCapsulePolyhedron (convexPolyhedrons, P0, P1, r);
	  convertCapsuleToSolverParam (initParam, P0, P1, r);

	  // Compute optimal capsule
	  fitter.computeBestFitCapsule (initParam);

	  // Display result
	  std::cout << "Initial: " << fitter.initParam () << std::endl;
	  std::cout << "Solution: " << fitter.solutionParam () << std::endl;
	}
      catch (boost::program_options::required_option& e)
	{
	  std::cerr << "Error: " << e.what() << std::endl << std::endl;

	  return EXIT_FAILURE;
	}
      catch (boost::program_options::error& e)
	{
	  std::cerr << "Error: " << e.what() << std::endl << std::endl;

	  return EXIT_FAILURE;
	}

    }
  catch (std::exception& e)
    {
      std::cerr << "Unhandled Exception reached the top of main: "
		<< e.what() << ", application will now exit" << std::endl;
      return EXIT_FAILURE;
    }

  return EXIT_SUCCESS;
}
