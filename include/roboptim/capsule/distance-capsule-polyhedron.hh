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
 * \brief Declaration of DistanceCapsulePolyhedron class that computes
 * the distance between a capsule and a polyhedron.
 */

#ifndef ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POLYHEDRON_HH
# define ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POLYHEDRON_HH

# include <kcd2/kcdInterface.h>

# include <kcd/poly-segment.hh>

# include <roboptim/core/function.hh>

namespace roboptim
{
  namespace capsule
  {
    /// \brief Import types from roboptim Function.
    typedef roboptim::Function::value_type value_type;
    typedef roboptim::Function::size_type size_type;
    typedef roboptim::Function::argument_t argument_t;
    
    /// \brief Define geometry and analsysis types.
    typedef CkcdPolyhedronShPtr polyhedron_t;
    typedef kcd::PolySegmentShPtr segment_t;
    typedef CkcdAnalysisShPtr analysis_t;

    /// \brief Distance to polyhedron function.
    ///
    /// This class computes the separation distance between a capsule
    /// and a polyhedron geometry.
    class DistanceCapsulePolyhedron
      : public roboptim::Function
    {
    public:
      /// \brief Constructor.
      ///
      /// \param polyhedron polyhedron that will be used in computing
      /// the separation distance to the capsule.
      DistanceCapsulePolyhedron (const polyhedron_t& polyhedron,
				 std::string name
				 = "distance to polyhedron") throw ();

      /// \brief Default constructor.
      DistanceCapsulePolyhedron (std::string name
				 = "distance to polyhedron") throw ();

      ~DistanceCapsulePolyhedron () throw ();

      /// \brief Get polyhedron attribute.
      virtual const polyhedron_t polyhedron () const throw ();

      /// \brief Set polyhedron attribute and update analysis
      /// attribute.
      virtual void polyhedron (const polyhedron_t& polyhedron) throw ();

    protected:
      /// \brief Compute the distance separating the capsule from the
      /// polyhedron.
      ///
      ///  An analysis between the polyhedron and the capsule axis (a
      ///  segment) is used. Returned distance will be positive if the
      ///  capsule and the polyhedron do not collide, and will
      ///  negative and equal to the penetration distance in case of
      ///  collision. Note that the analysis will fail if the segment
      ///  itself collides with the polyhedron.
      ///
      /// \param argument vector containing the capsule parameters. It
      /// contains in this order: the segment first end point
      /// coordinates, the segment second end point coordinates, the
      /// capsule radius.
      virtual void
      impl_compute (result_t& result,
		    const argument_t& argument) const throw ();

    private:
      /// \brief Polyhedron attribute.
      polyhedron_t polyhedron_;

      /// \brief Mutable segment attribute.
      mutable segment_t segment_;

      /// \brief Analysis attribute. It contains the two geometries
      /// that need to be analyzed and can return the distance
      /// separating them.
      analysis_t analysis_;
    };

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POLYHEDRON_HH
