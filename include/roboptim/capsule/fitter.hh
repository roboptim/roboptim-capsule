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
 * \brief Declaration of Fitter class that computes the best fitting
 * capsule for a polyhedron.
 */

#ifndef ROBOPTIM_CAPSULE_FITTER_HH
# define ROBOPTIM_CAPSULE_FITTER_HH

# include <roboptim/core/solver-factory.hh>
# include <roboptim/core/plugin/cfsqp.hh>

# include "roboptim/capsule/volume.hh"
# include "roboptim/capsule/distance-segment-polyhedron.hh"
# include "roboptim/capsule/distance-capsule-polyhedron.hh"

namespace roboptim
{
  namespace capsule
  {
    /// \brief Import type from CFSQP solver.
    typedef CFSQPSolver solver_t;

    /// \brief Capsule fitter class.
    ///
    /// This class computes the best fitting capsule over a
    /// polyhedron.
    class Fitter
    {
    public:
      /// \brief Constructor.
      Fitter (const polyhedron_t& polyhedron
	      = CkcdPolyhedron::create ()) throw ();

      ~Fitter () throw ();
      
      /// \brief Get polyhedron attribute.
      const polyhedron_t polyhedron () const throw ();

      /// \brief Set polyhedron attribute.
      void polyhedron (const polyhedron_t& polyhedron) throw ();

      /// \brief Get solution capsule parameters.
      const argument_t solutionParam () const throw ();

      /// \brief Compute best fitting capsule over polyhedron.
      ///
      /// Polyhedron attribute is used to compute capsule and set
      /// capsuleParam attribute.
      /// \param initParam initial capsule parameters
      void computeBestFitCapsule (const argument_t& initParam) throw ();
      
      /// \brief Compute best fitting capsule over polyhedron.
      /// 
      /// \param polyhedron Polyhedron over which the capsule is
      /// fitted
      /// \param initParam initial capsule parameters
      void computeBestFitCapsule (const polyhedron_t& polyhedron,
				  const argument_t& initParam) throw ();

      /// \brief Compute best fitting capsule over polyhedron.
      ///
      /// Polyhedron attribute is used to compute capsule and set
      /// capsuleParam attribute.
      /// 
      /// \param initParam initial capsule parameters
      /// \return capsule parameters
      const argument_t computeBestFitCapsuleParam (const argument_t&
						   initParam) throw ();

      /// \brief Compute best fitting capsule over polyhedron.
      ///
      /// \param polyhedron Polyhedron over which the capsule is
      /// fitted
      ///
      /// \param polyhedron Polyhedron over which the capsule is
      /// fitted
      /// \param initParam initial capsule parameters
      /// \return capsule parameters
      const argument_t computeBestFitCapsuleParam (const polyhedron_t&
						   polyhedron,
						   const argument_t&
						   initParam) throw ();

    protected:
      /// \brief Implementation of best fitting capsule computation.
      /// \param polyhedron Polyhedron over which the capsule is
      /// fitted
      /// \param initParam initial capsule parameters
      /// \return solutionParam solution capsule parameters
      void impl_computeBestFitCapsuleParam (const polyhedron_t& polyhedron,
					    const argument_t& initParam,
					    argument_t& solutionParam) throw ();

    private:
      /// \brief Polyhedron attribute.
      polyhedron_t polyhedron_;

      /// \brief Capsule solution parameters attribute.
      argument_t solutionParam_;
    };

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_FITTER_HH
