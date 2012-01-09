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

#define BOOST_TEST_MODULE fitter

#include <boost/test/unit_test.hpp>
#include <boost/test/output_test_stream.hpp>

#include <KineoModel/kppLicense.h>

#include "kcd/test-tree-segment.hh"
#include "kcd/detector-segment-obb.hh"
#include "kcd/detector-obb-segment.hh"
#include "kcd/detector-segment-triangle.hh"
#include "kcd/detector-triangle-segment.hh"

#include "roboptim/capsule/fitter.hh"

using boost::test_tools::output_test_stream;

BOOST_AUTO_TEST_CASE (fitter)
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

  // Create fitter. It is used to find the best fitting capsule on the
  // polyhedron.

  Fitter fitter (polyhedron);

  // Define initial capsule parameters. The segment must be inside the
  // polyhedron, and the capsule must contain the polyhedron.
  argument_t initParam (7);

  // First end point.
  initParam[0] = 0.;
  initParam[1] = 0.;
  initParam[2] = - halfLength / 4;
  // Second end point.
  initParam[3] = 0.;
  initParam[4] = 0.;
  initParam[5] = halfLength / 4;
  // Radius.
  initParam[6] = 100 * halfLength;
  
  std::cout << "initial parameters" << std::endl
	    << initParam << std::endl;

  // Compute best fitting capsule.
  fitter.computeBestFitCapsule (initParam);
  std::cout << "solution parameters" << std::endl
	    << fitter.solutionParam () << std::endl;
}
