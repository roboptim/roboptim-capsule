// Copyright (C) 2014 by Benjamin Chretien, CNRS-LIRMM.
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

#define BOOST_TEST_MODULE util

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <roboptim/core/io.hh>

#include "roboptim/capsule/util.hh"

using boost::test_tools::output_test_stream;

#define BOOST_CHECK_SMALL_OR_CLOSE(EXP, OBS, TOL) \
  if (std::fabs (EXP) < TOL) { \
    BOOST_CHECK_SMALL(OBS, TOL); \
  } else { \
    BOOST_CHECK_CLOSE(EXP, OBS, TOL); \
  }

BOOST_AUTO_TEST_CASE (util)
{
  using namespace roboptim::capsule;

  value_type epsilon = 1e-6;

  point_t x (1., 0., 0.);
  point_t y (0., 1., 0.);
  point_t z (0., 0., 1.);

  point_t a = point_t::Random ();
  point_t b = point_t::Random ();

  vector3_t dir_x = (b - a).normalized ();
  vector3_t dir_y = (dir_x.cross (z)).normalized ();
  if (dir_y.norm () < 1e-6)
    dir_y = (dir_x.cross (y)).normalized ();
  vector3_t dir_z = (dir_x.cross (dir_y)).normalized ();

  point_t p0 = a - dir_x;
  point_t p1 = b + dir_x;
  point_t p2 = (a + b)/2.;
  point_t p3 = p2 + dir_y;
  point_t p4 = a + 0.25 * (dir_x + dir_y);

  BOOST_CHECK_SMALL_OR_CLOSE (distancePointToSegment (p0, a, b), 1., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE (distancePointToSegment (p1, a, b), 1., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE (distancePointToSegment (p2, a, b), 0., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE (distancePointToSegment (p3, a, b), 1., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE (distancePointToSegment (p4, a, b), 0.25, epsilon);

  BOOST_CHECK_SMALL_OR_CLOSE ((a  - projectionOnSegment (p0, a, b)).norm (), 0., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE ((b  - projectionOnSegment (p1, a, b)).norm (), 0., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE ((p2 - projectionOnSegment (p2, a, b)).norm (), 0., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE ((p2 - projectionOnSegment (p3, a, b)).norm (), 0., epsilon);
  BOOST_CHECK_SMALL_OR_CLOSE (((a + 0.25 * dir_x) - projectionOnSegment (p4, a, b)).norm (), 0., epsilon);
}
