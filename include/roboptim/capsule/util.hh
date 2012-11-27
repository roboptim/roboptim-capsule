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

# include <geometric-tools/Wm5Capsule3.h>
# include <geometric-tools/Wm5ContCapsule3.h>

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

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_UTIL_HH_
