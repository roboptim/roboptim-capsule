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
 * \file src/distance-capsule-point.cc
 *
 * \brief Implementation of DistanceCapsulePoint.
 */

#ifndef ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POINT_CC_
# define ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POINT_CC_

# include <roboptim/capsule/distance-capsule-point.hh>

namespace roboptim
{
  namespace capsule
  {
    // -------------------PUBLIC FUNCTIONS-----------------------

    DistanceCapsulePoint::
    DistanceCapsulePoint (const point_t& point,
			  std::string name) throw ()
      : roboptim::DifferentiableFunction (7, 1, name),
	point_ (point)
    {
    }

    DistanceCapsulePoint::
    ~DistanceCapsulePoint () throw ()
    {
    }

    const point_t DistanceCapsulePoint::
    point () const throw ()
    {
      return point_;
    }

    // -------------------PROTECTED FUNCTIONS--------------------

    void DistanceCapsulePoint::
    impl_compute (result_t & result,
		  const argument_t & argument) const throw ()
    {
      assert (argument.size () == 7 && "Wrong argument size, expected 7.");

      result.setZero ();

      // Define capsule axis from argument.
      point_t endPoint1 (argument[0], argument[1], argument[2]);
      point_t endPoint2 (argument[3], argument[4], argument[5]);
      segment_t segment (endPoint1, endPoint2);

      // Compute distance between segment and point.
      value_type distance = segment.distance (point_);

      // Return difference between distance and capsule radius.
      result[0] = distance - argument[6];

      return;
    }

    void DistanceCapsulePoint::
    impl_gradient (gradient_t& gradient,
		   const argument_t& argument,
		   size_type functionId) const throw ()
    {
      assert (argument.size () == 7 && "Wrong argument size, expected 7.");

      gradient.setZero ();

      // Define capsule axis from argument.
      point_t endPoint1 (argument[0], argument[1], argument[2]);
      point_t endPoint2 (argument[3], argument[4], argument[5]);
      segment_t segment (endPoint1, endPoint2);

      // Compute distance between segment and point.
      value_type d = segment.distance (point_);
      point_t segmentClosest = segment.projection (point_);

      // Compute unit axis between closest points.
      point_t unitPoint = segmentClosest - point_;
      unitPoint.normalize ();
      vector_t unit (3);
      unit[0] = unitPoint[0];
      unit[1] = unitPoint[1];
      unit[2] = unitPoint[2];

      // Compute closest point parameter.
      point_t segmentClosestFromEndP1 = segmentClosest - endPoint1;
      point_t endP2FromEndP1 = endPoint2 - endPoint1;
      value_type lambda;
      if (endP2FromEndP1.norm () > 0)
	lambda = segmentClosestFromEndP1.norm () / endP2FromEndP1.norm ();
      else
	lambda = 0.;

      // Compute closest point jacobian wrt parameters.
      matrix_t jacobian (3, 7);
      jacobian.setZero ();
      jacobian(0,0) = 1 - lambda;
      jacobian(1,1) = 1 - lambda;
      jacobian(2,2) = 1 - lambda;
      jacobian(0,3) = lambda;
      jacobian(1,4) = lambda;
      jacobian(2,5) = lambda;

      // Compute capsule-point distance gradient.
      // TODO: double-check this
      vector_t segmentDistanceGradient = unit.transpose () * jacobian;

      // Compute radius gradient.
      vector_t radiusGradient (7);
      radiusGradient.setZero ();
      radiusGradient[6] = 1.;

      // Return capsule distance gradient.
      gradient = segmentDistanceGradient - radiusGradient;

      return;
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POINT_CC_
