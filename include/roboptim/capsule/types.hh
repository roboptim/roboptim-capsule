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


#ifndef ROBOPTIM_CAPSULE_FWD_HH_
# define ROBOPTIM_CAPSULE_FWD_HH_

# include <Eigen/Core>
# include <Eigen/Geometry>

# include <roboptim/core/function.hh>
# include <roboptim/core/solver.hh>
# include <roboptim/core/plugin/ipopt.hh>

namespace roboptim
{
  namespace capsule
  {
    /// \brief Import types from roboptim Function.
    typedef roboptim::Function::value_type        value_type;
    typedef roboptim::Function::size_type         size_type;
    typedef roboptim::Function::argument_t        argument_t;
    typedef roboptim::Function::vector_t          vector_t;
    typedef roboptim::Function::matrix_t          matrix_t;

    /// \brief Import solver type.
    typedef roboptim::Solver<
              roboptim::DifferentiableFunction,
              boost::mpl::vector<
                roboptim::LinearFunction,
                roboptim::DifferentiableFunction> > solver_t;

    /// \brief Define geometry types.
    typedef Eigen::Matrix<value_type,3,1>         point_t;
    typedef Eigen::Matrix<value_type,3,1>         vector3_t;
    typedef std::vector<point_t>                  polyhedron_t;
    typedef std::vector<polyhedron_t>             polyhedrons_t;
    typedef Eigen::ParametrizedLine<value_type,3> segment_t;

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_FWD_HH_
