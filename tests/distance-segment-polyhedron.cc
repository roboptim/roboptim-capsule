// Copyright (C) 2012 by Antonio El Khoury.
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

#define BOOST_TEST_MODULE distance-segment-polyhedron

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "kcd/test-tree-segment.hh"
#include "kcd/detector-segment-obb.hh"
#include "kcd/detector-obb-segment.hh"
#include "kcd/detector-segment-triangle.hh"
#include "kcd/detector-triangle-segment.hh"

#include <roboptim/core/io.hh>
#include <roboptim/core/finite-difference-gradient.hh>

#include "roboptim/capsule/distance-segment-polyhedron.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (distance_segment_polyhedron)
{
  using namespace roboptim::capsule;
  using namespace kcd;

  // Validate Kineo license.
  if (!CkppLicense::initialize ())
    {
      std::cout << "Failed to validate Kineo license." << std::endl;
      return;
    }

  // Register segment test tree and detector.
  CkcdGlobal::instance ().registerTestTreeLocked
    (&CkcdGlobal::createTestTreeLocked<TestTreeSegment>);
  CkcdGlobal::instance ().registerDetector
    (&CkcdGlobal::createDetector<DetectorSegmentOBB>);
  CkcdGlobal::instance ().registerDetector
    (&CkcdGlobal::createDetector<DetectorOBBSegment>);
  CkcdGlobal::instance ().registerDetector
    (&CkcdGlobal::createDetector<DetectorSegmentTriangle>);
  CkcdGlobal::instance ().registerDetector
    (&CkcdGlobal::createDetector<DetectorTriangleSegment>);

  // Build cubic polyhedron.
  CkcdPolyhedronShPtr polyhedron = CkcdPolyhedron::create ();
  unsigned int rank;
  kcdReal halfLength = 0.5f;

  polyhedron->addPoint(-halfLength, -halfLength, -halfLength, rank);
  polyhedron->addPoint(-halfLength, -halfLength, halfLength, rank);
  polyhedron->addPoint(-halfLength, halfLength, -halfLength, rank);
  polyhedron->addPoint(-halfLength, halfLength, halfLength, rank);
  polyhedron->addPoint(halfLength, -halfLength, -halfLength, rank);
  polyhedron->addPoint(halfLength, -halfLength, halfLength, rank);
  polyhedron->addPoint(halfLength, halfLength, -halfLength, rank);
  polyhedron->addPoint(halfLength, halfLength, halfLength, rank);

  polyhedron->reserveNTriangles(12);
  polyhedron->addTriangle(3,7,5, rank);
  polyhedron->addTriangle(3,5,1, rank);
  polyhedron->addTriangle(2,3,1, rank);
  polyhedron->addTriangle(2,1,0, rank);
  polyhedron->addTriangle(6,2,0, rank);
  polyhedron->addTriangle(6,0,4, rank);
  polyhedron->addTriangle(7,6,4, rank);
  polyhedron->addTriangle(7,4,5, rank);
  polyhedron->addTriangle(2,6,7, rank);
  polyhedron->addTriangle(2,7,3, rank);
  polyhedron->addTriangle(1,5,4, rank);
  polyhedron->addTriangle(1,4,0, rank);

  polyhedron->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

  // Create distance function.
  DistanceSegmentPolyhedron distanceFunction (polyhedron,
					      "distance segment to polyhedron");

  // Compute distance for segment given by argument.
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
  argument[6] = 0.1;

  // Translate polyhedron on the x axis and compute distance.
  kcdReal distance = halfLength + 0.01f;
  while (distance < 2.0f)
    {
      polyhedron->setAbsolutePosition (CkcdMat4 ().translate (distance, 0.f, 0.f));
      BOOST_CHECK_CLOSE (distanceFunction (argument)[0],
			 distance - halfLength,
			 1e-4);
      distance += 0.01f;
    }

  // Create finite difference gradient function with distance
  // function.
  roboptim::FiniteDifferenceGradient<> fdgDistanceFunction (distanceFunction);
  BOOST_CHECK_EQUAL (checkGradient (fdgDistanceFunction, 0, argument), true);
}
