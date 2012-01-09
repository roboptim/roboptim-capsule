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
 * \file src/distance-segment-polyhedron.cc
 *
 * \brief Implementation of DistanceSegmentPolyhedron.
 */

#ifndef ROBOPTIM_SEGMENT_DISTANCE_SEGMENT_POLYHEDRON_CC_
# define ROBOPTIM_SEGMENT_DISTANCE_SEGMENT_POLYHEDRON_CC_

# include <math.h>

# include "roboptim/capsule/distance-segment-polyhedron.hh"

namespace roboptim
{
  namespace capsule
  {
    // -------------------PUBLIC FUNCTIONS-----------------------

    DistanceSegmentPolyhedron::
    DistanceSegmentPolyhedron (const polyhedron_t& polyhedron,
			       std::string name) throw ()
      : roboptim::Function (7, 1, name),
	polyhedron_ (polyhedron)
    {
      assert (!!polyhedron && "Null pointer to polyhedron.");

      // Create emtpy segment.
      segment_ = kcd::PolySegment::create ();

      // Create analysis and add geometries in it.
      analysis_ = CkcdAnalysis::create ();
      analysis_->leftObject (segment_);
      analysis_->rightObject (polyhedron);
      analysis_->analysisData ()->analysisType (CkcdAnalysisType::EXACT_DISTANCE);
    }

    DistanceSegmentPolyhedron::
    DistanceSegmentPolyhedron (std::string name) throw ()
      : roboptim::Function (7, 1, name),
	segment_ (),
	polyhedron_ (),
	analysis_ ()
    {
    }

    DistanceSegmentPolyhedron::
    ~DistanceSegmentPolyhedron () throw ()
    {
    }

    const polyhedron_t DistanceSegmentPolyhedron::
    polyhedron () const throw ()
    {
      return polyhedron_;
    }

    void DistanceSegmentPolyhedron::
    polyhedron (const polyhedron_t& polyhedron) throw ()
    {
      assert (!!polyhedron && "Null pointer to polyhedron.");

      polyhedron_ = polyhedron;
      analysis_->rightObject (polyhedron);
    }

    // -------------------PROTECTED FUNCTIONS--------------------

    void DistanceSegmentPolyhedron::
    impl_compute (result_t & result,
		  const argument_t & argument) const throw ()
    {
      assert (argument.size () == 7 && "Wrong argument size, expected 7.");

      result.clear ();

      // Define segment polygon from argument.
      CkcdPoint endPoint1 (argument[0], argument[1], argument[2]);
      CkcdPoint endPoint2 (argument[3], argument[4], argument[5]);

      segment_->setSegment (0, endPoint1, endPoint2);
      segment_->makeCollisionEntity (CkcdObject::IMMEDIATE_BUILD);

      // Compute distance.
      analysis_->compute ();

      if (analysis_->countExactDistanceReports () == 1)
	result[0] = analysis_->exactDistanceReport (0)->distance ();
      else
	result[0] = 0.;

      std::cout << result[0] << std::endl;

      return;
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_SEGMENT_DISTANCE_SEGMENT_POLYHEDRON_CC_
