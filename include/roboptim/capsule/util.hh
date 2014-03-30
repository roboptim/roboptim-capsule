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


#ifndef ROBOPTIM_CAPSULE_UTIL_HH_
# define ROBOPTIM_CAPSULE_UTIL_HH_

# include <iostream>
# include <set>
# include <limits>

# include <boost/foreach.hpp>

# include <roboptim/capsule/fwd.hh>
# include <roboptim/capsule/types.hh>
# include <roboptim/capsule/qhull.hh>

namespace roboptim
{
  namespace capsule
  {

    /// Creates a convex hull from a set of points.
    polyhedron_t convexHullFromPoints (const std::vector<point_t>& points)
    {
      polyhedron_t convexPolyhedron;

# ifdef HAVE_QHULL
      int numpoints = static_cast<int> (points.size ());
      int dim = 3;

      std::vector<value_type> rboxpoints (dim * numpoints);
      size_t iter = 0;

      for (std::vector<point_t>::const_iterator
	     point = points.begin (); point != points.end (); ++point)
	{
	  rboxpoints[iter++] = (*point)[0];
	  rboxpoints[iter++] = (*point)[1];
	  rboxpoints[iter++] = (*point)[2];
	}

      // Compute the convex hull with qhull
      char flags[25];
      sprintf (flags, "qhull Qc Qt Qi");
      int exitcode = qh_new_qhull (dim, numpoints,
				   rboxpoints.data (), 0,
				   flags, NULL, NULL);

      int num_qhull_vertices = qh num_vertices;
      assert (num_qhull_vertices > 0
	      && "Qhull computation failed (empty vector).");

      if (exitcode != 0 || num_qhull_vertices == 0)
	{
	  // Return empty polyhedron
	  return convexPolyhedron;
	}

      // Get the list of points
      convexPolyhedron.resize (num_qhull_vertices);
      vertexT* vertex = qh vertex_list;

      FORALLvertices {
	// qh_pointid (vertex->point) is the point id of the vertex
	int id = qh_pointid (vertex->point);
	// vertex->point is the coordinates of the vertex
	convexPolyhedron[id] = point_t (vertex->point[0],
					vertex->point[1],
					vertex->point[2]);
      }

      qh_freeqhull (!qh_ALL);
# else
      std::cerr << "Qhull not found, cannot compute the convex hull."
		<< std::endl;
# endif //! HAVE_QHULL
      // Return the convex hull as a polyhedron
      return convexPolyhedron;
    }


    /// \brief Structure containing Capsule data (start point, end point and
    // radius).
    struct Capsule
    {
      point_t P0, P1;
      value_type radius;
    };

    /// \brief Distance from a point to a line described as a point and a
    // direction.
    inline value_type distancePointToLine (const point_t& point,
                                           const point_t& linePoint,
                                           const vector3_t& dir)
    {
      return (dir.cross (linePoint - point)).norm () / dir.norm ();
    }

    /// \brief Compute the covariance matrix of a set of points.
    inline Eigen::Matrix3d covarianceMatrix (const std::vector<point_t>& points)
    {
      value_type oon = 1.0 / (value_type)points.size();
      point_t c(0., 0., 0.);
      value_type e00, e11, e22, e01, e02, e12;

      // compute the center of mass of the points
      for (size_t i = 0; i < points.size (); ++i)
	c += points[i];
      c *= oon;

      // compute covariance elements
      e00 = e11 = e22 = e01 = e02 = e12 = 0.0;
      for (size_t i = 0; i < points.size (); ++i)
        {
	  // translate points so center of mass is at origin
	  point_t p = points[i] - c;
	  // compute covariance of translated points
	  e00 += p[0] * p[0];
	  e11 += p[1] * p[1];
	  e22 += p[2] * p[2];
	  e01 += p[0] * p[1];
	  e02 += p[0] * p[2];
	  e12 += p[1] * p[2];
        }

      // fill in the covariance matrix elements
      Eigen::Matrix3d cov;

      cov (0,0) = e00 * oon;
      cov (1,1) = e11 * oon;
      cov (2,2) = e22 * oon;
      cov (0,1) = cov (1,0) = e01 * oon;
      cov (0,2) = cov (2,0) = e02 * oon;
      cov (1,2) = cov (2,1) = e12 * oon;

      return cov;
    }


    // Returns indices imin and imax into pt[] array of the least and
    // most, respectively, distant points along the direction dir
    inline void extremePointsAlongDirection (vector3_t dir,
                                             const std::vector<point_t>& points,
                                             int& imin, int& imax)
    {
      double minproj = std::numeric_limits<double>::max ();
      double maxproj = -minproj;

      for (size_t i = 0; i < points.size(); ++i)
        {
	  // Project vector from origin to point onto direction vector
	  double proj = points[i].dot (dir);
	  // Keep track of least distant point along direction vector
	  if (proj < minproj) {
	    minproj = proj;
	    imin = static_cast<int> (i);
	  }
	  // Keep track of most distant point along direction vector
	  if (proj > maxproj) {
	    maxproj = proj;
	    imax = static_cast<int> (i);
	  }
        }
    }

    /// Computes a capsule from a set of points.
    /// The algorithm currently used relies on the search of the largest spread
    /// direction (PCA).
    /// TODO: Optimize computation speed.
    /// Spheres on both ends do not contain any point yet, cylinder length
    /// could be shortened to have a better fit.
    inline Capsule capsuleFromPoints (const std::vector<point_t>& points)
    {
      assert (points.size () > 0
              && "Cannot compute capsule for empty polyhedron.");

      // Create the covariance matrix for PCA
      Eigen::Matrix3d covariance = covarianceMatrix (points);

      // Compute eigenvectors and eigenvalues
      Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es;
      es.compute (covariance,Eigen::ComputeEigenvectors);

      Eigen::Matrix3d eigenVectors = es.eigenvectors ();
      Eigen::Matrix3d eigenValues = es.eigenvalues ().asDiagonal ();

      // Find the largest eigenvalue and the corresponding direction
      // (largest spread)
      unsigned maxc, minc;
      maxc = minc = 0;
      value_type absev, maxev, minev;
      maxev = minev = std::fabs (eigenValues (0,0));

      if ((absev = std::fabs (eigenValues (1,1))) > maxev)
        {
	  maxc = 1;
	  maxev = absev;
        }
      else
        {
	  minc = 1;
	  minev = absev;
        }

      if ((absev = std::fabs (eigenValues (2,2))) > maxev)
        {
	  maxc = 2;
	  maxev = absev;
        }
      else if (minev > absev)
        {
	  minc = 2;
	  minev = absev;
        }

      vector3_t dirLargestSpread = eigenVectors.col (maxc);
      dirLargestSpread.normalize ();

      // Find the most extreme points along the largest spread direction.
      // Those points will help to find the length of the capsule.
      int iminLargestSpread = 0;
      int imaxLargestSpread = 0;
      extremePointsAlongDirection (dirLargestSpread, points,
                                   iminLargestSpread, imaxLargestSpread);
      point_t minptLargestSpread = points[iminLargestSpread];
      point_t maxptLargestSpread = points[imaxLargestSpread];

      // Compute the start point
      // The cylinder axis will be (average point, largest spread direction).
      // However, a better point could be found with a more complicated
      // algorithm, thus reducing the volume of the capsule.
      point_t average;
      for (size_t i = 0; i < points.size (); ++i)
        {
	  average += points[i];
        }
      average /= static_cast<value_type> (points.size ());

      // Find the correct radius for the capsule.
      value_type radius = 0;
      for (size_t i = 0; i < points.size (); ++i)
        {
	  value_type dist = distancePointToLine
	    (points[i], average, dirLargestSpread);
	  if (dist > radius) radius = dist;
        }

      // Find the correct length for the capsule
      value_type length = (maxptLargestSpread - minptLargestSpread).norm ();

      // Length used to find the correct center position on the direction axis
      value_type maxLengthFromAverage
	= std::fabs((maxptLargestSpread - average).dot (dirLargestSpread));
      point_t center = average + (maxLengthFromAverage - 0.5 * length)
	* dirLargestSpread;

      // Optimization of the volume
      // - We determine the points located at
      //   both extremities (+/-)(0.5 * length - radius)
      // - For all of those points, we look for the start/endpoint position
      //   that will minimize the capsule volume.
      std::vector<point_t> nearStartPoints;
      std::vector<point_t> nearEndPoints;
      point_t start = center - (0.5 * length - radius) * dirLargestSpread;
      point_t end = center + (0.5 * length - radius) * dirLargestSpread;

      for(size_type i = 0; i < points.size (); ++i)
        {
	  // if located near the start boundary
	  value_type dirDist = dirLargestSpread.dot (points[i] - center);
	  if (-dirDist >  0.5 * length - radius)
	    nearStartPoints.push_back(points[i]);
	  // else if located near the end boundary
	  else if (dirDist > 0.5 * length - radius)
	    nearEndPoints.push_back(points[i]);
        }

      // we move the position of the start point to include all points in its
      // vicinity
      for (size_type i = 0; i < nearStartPoints.size (); ++i)
        {
	  if ((nearStartPoints[i] - start).norm () > radius)
            {
	      // using pythagore theorem
	      value_type h = distancePointToLine (nearStartPoints[i],
						  center, dirLargestSpread);
	      value_type l = (nearStartPoints[i] - start)
		.dot (-dirLargestSpread);
	      if (l - sqrt(radius * radius - h * h) > 0)
		start -= (l - sqrt(radius * radius - h * h))
		  * dirLargestSpread;
            }
        }

      // we move the position of the end point to include all points in its
      // vicinity
      for (size_type i = 0; i < nearEndPoints.size (); ++i)
        {
	  if ((nearEndPoints[i] - end).norm () > radius)
            {
	      // using pythagore theorem
	      value_type l = (nearEndPoints[i] - end).dot (dirLargestSpread);
	      value_type h = distancePointToLine (nearEndPoints[i],
						  center, dirLargestSpread);
	      if (l - std::sqrt (radius * radius - h * h) > 0)
		end += (l - std::sqrt (radius * radius - h * h))
		  * dirLargestSpread;
            }
        }

      Capsule capsule;
      capsule.P0 = start;
      capsule.P1 = end;
      capsule.radius = radius;

      return capsule;
    }


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
      size_type nbPoints = 0;
      BOOST_FOREACH (polyhedron_t polyhedron, polyhedrons)
	{
          nbPoints += polyhedron.size ();
	}

      std::vector<point_t> points (nbPoints);
      size_type k = 0;
      BOOST_FOREACH (polyhedron_t polyhedron, polyhedrons)
	{
	  BOOST_FOREACH (point_t point, polyhedron)
	    {
	      points[k++] = point;
	    }
	}

      // Compute bounding capsule of points.
      Capsule capsule = capsuleFromPoints (points);

      // Get capsule parameters.
      endPoint1 = capsule.P0;
      endPoint2 = capsule.P1;
      radius = capsule.radius;
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

      assert (polyhedron.size() > 0
	      && "Polyhedron merging failed.");

      // Build convex polyhedron that contains unique points.
      polyhedron_t convexPolyhedron = convexHullFromPoints (polyhedron);

      assert (convexPolyhedron.size() > 0
	      && "Convex polyhedron computation failed.");

      convexPolyhedrons.push_back (convexPolyhedron);
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_UTIL_HH_
