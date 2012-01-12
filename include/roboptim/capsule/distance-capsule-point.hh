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
 * \brief Declaration of DistanceSegmentPoint class that computes
 * the distance between a segment and a point.
 */

#ifndef ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POINT_HH
# define ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POINT_HH

# include <kcd2/kcdInterface.h>

# include <roboptim/core/differentiable-function.hh>

namespace roboptim
{
  namespace capsule
  {
    /// \brief Import types from roboptim Function.
    typedef roboptim::Function::value_type value_type;
    typedef roboptim::Function::size_type size_type;
    typedef roboptim::Function::argument_t argument_t;
    typedef roboptim::Function::vector_t vector_t;
    typedef roboptim::Function::matrix_t matrix_t;

    /// \brief Define point type.
    typedef CkcdPoint point_t;

    /// \brief Distance to point RobOptim function.
    class DistanceCapsulePoint
      : public roboptim::DifferentiableFunction
    {
    public:
      /// \brief Constructor.
      ///
      /// \param point point that will be used in computing distance
      /// between the capsule and the point.
      DistanceCapsulePoint (const point_t& point,
			    std::string name
			    = "distance to point") throw ();

      ~DistanceCapsulePoint () throw ();

      /// \brief Get point attribute.
      virtual const point_t point () const throw ();

    protected:
      /// \brief Computes the distance from capsule to a point.
      ///
      /// If the result is negative, the point is inside the capsule,
      /// otherwise it is outside the capsule.
      ///
      /// \param argument vector containing the capsule parameters. It
      /// contains in this order: the segment first end point
      /// coordinates, the segment second end point coordinates, the
      /// capsule radius.
      virtual void
      impl_compute (result_t& result,
		    const argument_t& argument) const throw ();

      /// \brief Compute of the distance gradient with respect to the
      /// capsule parameters.
      ///
      /// \param argument vector containing the capsule parameters. It
      /// contains in this order: the segment first end point
      /// coordinates, the segment second end point coordinates, the
      /// capsule radius.
      virtual void
      impl_gradient (gradient_t& gradient,
		     const argument_t& argument,
		     size_type functionId = 0) const throw ();

    private:
      /// \brief Point attribute.
      point_t point_;
    };

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_DISTANCE_CAPSULE_POINT_HH
