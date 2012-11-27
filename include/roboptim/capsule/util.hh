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


#ifndef ROBOPTIM_CAPSULE_UTIL_HH_
# define ROBOPTIM_CAPSULE_UTIL_HH_

# include <iostream>

# include <boost/foreach.hpp>

# include <geometric-tools/Wm5Capsule3.h>
# include <geometric-tools/Wm5ContCapsule3.h>
# include <geometric-tools/Wm5ConvexHull3.h>

# include <roboptim/capsule/fwd.hh>
# include <roboptim/capsule/types.hh>

namespace roboptim
{
  namespace capsule
  {
    /// \brief Convert Capsule parameters to RobOptim solver
    /// parameters vector.
    ///
    /// \param endPoint1 capsule axis first end point
    /// \param endPoint2 capsule axis second end point
    /// \param radius capsule radius
    /// \return dst parameters vector containing, in this
    /// order, the capsule axis first end point coordinates, the
    /// capsule axis second end point coordinates and the radius.
    inline void convertCapsuleToSolverParam (argument_t& dst,
					     const point_t& endPoint1,
					     const point_t& endPoint2,
					     const value_type& radius)
    {
      dst.resize (7);

      dst[0] = endPoint1[0];
      dst[1] = endPoint1[1];
      dst[2] = endPoint1[2];
      dst[3] = endPoint2[0];
      dst[4] = endPoint2[1];
      dst[5] = endPoint2[2];
      dst[6] = radius;
    }

    /// \brief Convert RobOptim solver parameters vector to Capsule
    /// parameters.
    ///
    /// \param src parameters vector containing, in this
    /// order, the capsule axis first end point coordinates, the
    /// capsule axis second end point coordinates and the radius.
    /// \return endPoint1 capsule axis first end point
    /// \return endPoint2 capsule axis second end point
    /// \return radius capsule radius
    inline void convertSolverParamToCapsule (point_t& endPoint1,
					     point_t& endPoint2,
					     value_type& radius,
					     const argument_t src)
    {
      assert (src.size () == 7 && "Incorrect src size, expected 7.");
      assert (src[6] > 0
	      && "Invalid value for radius, expected non-negative value.");

      endPoint1[0] = src[0];
      endPoint1[1] = src[1];
      endPoint1[2] = src[2];
      endPoint2[0] = src[3];
      endPoint2[1] = src[4];
      endPoint2[2] = src[5];
      radius = src[6];
    }

    /// \brief Convert a polyhedron vector to a single polyhedron.
    ///
    /// Result polyhedron is the union of all polyhedrons.
    ///
    /// \param polyhedrons polyhedron vector containing all
    /// polyhedrons.
    ///
    /// \return polyhedron union polyhedron
    inline void
    convertPolyhedronVectorToPolyhedron (polyhedron_t& polyhedron,
					 const polyhedrons_t& polyhedrons)
    {
      assert (polyhedrons.size () !=0 && "Empty polyhedron vector.");
      assert (polyhedron.size () == 0 && "Union polyhedron must be empty.");

      BOOST_FOREACH (polyhedron_t poly, polyhedrons)
	{
	  BOOST_FOREACH (point_t point, poly)
	    polyhedron.push_back (point);
	}
    }

    /// \brief Compute bounding capsule of a vector of polyhedrons.
    ///
    /// Compute axis of capsule segment using least-squares fit. Radius
    /// is maximum distance from points to axis. Hemispherical caps are
    /// chosen as close together as possible.
    ///
    /// \param polyhedrons vector of polyhedrons that contain the
    /// points
    /// \return endPoint1 bounding capsule segment first end point
    /// \return endPoint2 bounding capsule segment second end point
    /// \return radius bounding capsule radius
    inline void
    computeBoundingCapsulePolyhedron (const polyhedrons_t& polyhedrons,
				      point_t& endPoint1,
				      point_t& endPoint2,
				      value_type& radius)
    {
      assert (polyhedrons.size () !=0 && "Empty polyhedron vector.");

      // Retrieve vector of points from polyhedrons.
      unsigned nbPoints = 0;
      for (unsigned i = 0; i < polyhedrons.size (); ++i)
	nbPoints += polyhedrons[i].size ();

      point_t points[nbPoints];
      unsigned k = 0;
      for (unsigned i = 0; i < polyhedrons.size (); ++i)
	{
	  polyhedron_t polyhedron = polyhedrons[i];
	  for (unsigned j = 0; j < polyhedron.size (); ++j)
	    {
	      points[k] = polyhedron[j];
	      ++k;
	    }
	}

      // Compute bounding capsule of points.
      Wm5::Capsule3<value_type> capsule = Wm5::ContCapsule (nbPoints, points);

      // Get capsule parameters.
      endPoint1 = capsule.Segment.P0;
      endPoint2 = capsule.Segment.P1;
      radius = capsule.Radius;
    }

    /// \brief Compute the convex polyhedron over a vector of
    /// polyhedrons.
    ///
    /// Compute the polygon representing the convex hull of the union
    /// of all polyhedrons in a vector, and store it in a one-element
    /// vector.
    ///
    /// \param polyhedrons vector of polyhedrons that contain the
    /// points
    ///
    /// \return convexPolyhedron vector of polyhedrons containing one
    /// element, i.e. the convex hull.
    inline void
    computeConvexPolyhedron (const polyhedrons_t& polyhedrons,
			     polyhedrons_t& convexPolyhedrons)
    {
      assert (polyhedrons.size () !=0 && "Empty polyhedron vector.");
      assert (convexPolyhedrons.size() == 0
	      && "Convex polyhedron vector must be empty.");

      polyhedron_t polyhedron;
      convertPolyhedronVectorToPolyhedron (polyhedron, polyhedrons);

      point_t points[polyhedron.size ()];
      for (unsigned i = 0; i < polyhedron.size (); ++i)
	points[i] = polyhedron[i];

      Wm5::ConvexHull3<value_type> convexHull
	(polyhedron.size (), points, 1e-8, false, Wm5::Query::QT_INT64);

      // Build convex polyhedron that contains unique points.
      polyhedron_t convexPolyhedron;
      std::set<int> addedVertices;
      for (unsigned i = 0; i < convexHull.GetNumSimplices (); ++i)
	{
	  for (unsigned j = 0; j < 3; ++j)
	    {
	      int index = convexHull.GetIndices ()[3*i+j];
	      if (addedVertices.find (index) == addedVertices.end ())
		{
		  point_t p (points[index]);
		  convexPolyhedron.push_back (p);
		  addedVertices.insert (index);
		}
	    }
	}

      convexPolyhedrons.push_back (convexPolyhedron);
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_UTIL_HH_
