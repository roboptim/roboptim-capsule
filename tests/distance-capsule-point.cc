// Copyright (C) 2012 by Antonio El Khoury, CNRS.
//
// This file is part of the roboptim-capsule.
//
// roboptim-capsule is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// roboptim-capsule is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with roboptim-capsule.  If not, see <http://www.gnu.org/licenses/>.

#define BOOST_TEST_MODULE distance-capsule-polyhedron

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/io.hh>
#include <roboptim/core/finite-difference-gradient.hh>

#include "roboptim/capsule/distance-capsule-point.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (distance_capsule_polyhedron)
{
  using namespace roboptim::capsule;

  // Build cubic polyhedron.
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

  // Compute distance for capsule given by argument.
  argument_t argument (7);

  // First end point.
  argument[0] = 0.1;
  argument[1] = 0.;
  argument[2] = 0.;
  // Second end point.
  argument[3] = -0.1;
  argument[4] = 0.;
  argument[5] = 0.;
  // Radius.
  argument[6] = sqrt (3) / 2;

  // Cycle throught all points in cube polyhedron.
  for (size_type i = 0; i < polyhedron.size (); ++i)
    {
      // Create distance function.
      point_t point = polyhedron[i];
      DistanceCapsulePoint distanceFunction (point,
					     "distance capsule to point");

      // Compute distance of capsule to point.
      std::cout << "distance to point " << point << std::endl;
      std::cout << distanceFunction (argument)[0] << std::endl;

      // Compute distance gradient function.
      roboptim::FiniteDifferenceGradient<>
	fdgDistanceFunction (distanceFunction, 1e-6);
      
      std::cout << "gradient" << std::endl;
      std::cout << distanceFunction.gradient (argument) << std::endl;
      std::cout << "finite difference gradient" << std::endl;
      std::cout << fdgDistanceFunction.gradient (argument) << std::endl;

      BOOST_CHECK_EQUAL (checkGradient (distanceFunction, 0, argument, 1e-6),
			 true);
    }
}
