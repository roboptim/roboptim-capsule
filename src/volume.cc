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
 * \file src/volume.cc
 *
 * \brief Implementation of Volume.
 */

#ifndef ROBOPTIM_CAPSULE_VOLUME_CC_
# define ROBOPTIM_CAPSULE_VOLUME_CC_

# include <math.h>

# include <roboptim/capsule/volume.hh>

namespace roboptim
{
  namespace capsule
  {
    // -------------------PUBLIC FUNCTIONS-----------------------

    Volume::
    Volume (std::string name) throw ()
      : roboptim::DifferentiableFunction (7, 1, name)
    {
    }

    Volume::
    ~Volume () throw ()
    {
    }

    // -------------------PROTECTED FUNCTIONS--------------------

    void Volume::
    impl_compute (result_t & result,
		  const argument_t & argument) const throw ()
    {
      assert (argument.size () == 7 && "Wrong argument size, expected 7.");

      result.clear ();

      // Compute capsule volume.
      value_type length = sqrt ((argument[0] - argument[3])
				* (argument[0] - argument[3])
				+ (argument[1] - argument[4])
				* (argument[1] - argument[4])
				+ (argument[2] - argument[5])
				* (argument[2] - argument[5]));

      result[0] = length * M_PI * argument[6] * argument[6]
	+ 4. / 3. * M_PI * argument[6] * argument[6] * argument[6];

      return;
    }

    void Volume::
    impl_gradient (gradient_t& gradient,
		   const argument_t& argument,
		   size_type functionId) const throw ()
    {
      assert (functionId == 0);
      assert (argument.size () == 7 &&  "Wrong argument size, expected 7.");

      gradient.clear ();

      value_type length = sqrt ((argument[0] - argument[3])
				* (argument[0] - argument[3])
				+ (argument[1] - argument[4])
				* (argument[1] - argument[4])
				+ (argument[2] - argument[5])
				* (argument[2] - argument[5]));

      gradient[0] = 1 / length * (argument[0] - argument[3])
      	* M_PI * argument[6] * argument[6];

      gradient[1] = 1 / length * (argument[1] - argument[4])
	* M_PI * argument[6] * argument[6];

      gradient[2] = 1 / length * (argument[2] - argument[5])
	* M_PI * argument[6] * argument[6];

      gradient[3] = 1 / length * (argument[3] - argument[0])
	* M_PI * argument[6] * argument[6];

      gradient[4] = 1 / length * (argument[4] - argument[1])
	* M_PI * argument[6] * argument[6];

      gradient[5] = 1 / length * (argument[5] - argument[2])
	* M_PI * argument[6] * argument[6];

      gradient[6] = length * 2 * M_PI * argument[6]
      	+ 4 * M_PI * argument[6] * argument[6];

      return;
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_VOLUME_CC_
