// Copyright (C) 2011 by Antonio El Khoury.
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

#define BOOST_TEST_MODULE capsule-volume

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/io.hh>
#include <roboptim/core/finite-difference-gradient.hh>

#include "roboptim/capsule/volume.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (capsule_volume)
{
  using namespace roboptim::capsule;

  // Create volume function.
  Volume volumeFunction ("capsule volume");

  // Compute volume for capsule given by argument.
  argument_t argument (7);
  
  // First end point.
  argument[0] = 0.;
  argument[1] = 0.;
  argument[2] = -1.;
  // Second end point.
  argument[3] = 0.;
  argument[4] = 0.;
  argument[5] = 1.;
  // Radius.
  argument[6] = 1.;

  double volume = volumeFunction (argument)[0];

  BOOST_CHECK_CLOSE (volume, 10. * M_PI / 3., 1e-5);

  // Compute volume gradient and check value.
  bool isGoodGradient = checkGradient (volumeFunction, 0, argument);
  BOOST_CHECK_EQUAL (isGoodGradient, true);
}
