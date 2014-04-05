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
 * \brief Declaration of Fitter class that computes the best fitting
 * capsule for a polyhedron.
 */

#ifndef ROBOPTIM_CAPSULE_FITTER_HH
# define ROBOPTIM_CAPSULE_FITTER_HH

# include <roboptim/core/solver-factory.hh>

# include <roboptim/capsule/types.hh>
# include <roboptim/capsule/volume.hh>
# include <roboptim/capsule/distance-capsule-point.hh>

namespace roboptim
{
  namespace capsule
  {
    /// \brief Capsule fitter class.
    ///
    /// This class computes the best fitting capsule over a
    /// polyhedron.
    class Fitter
    {
    public:
      /// \brief Constructor.
      Fitter (const polyhedrons_t& polyhedrons,
              std::string solver = "ipopt") throw ();

      ~Fitter () throw ();

      /// \brief Get polyhedron attribute.
      const polyhedrons_t polyhedrons () const throw ();

      /// \brief Set polyhedron attribute.
      void polyhedrons (const polyhedrons_t& polyhedrons) throw ();

      /// \brief Get capsule volume for initial parameters.
      value_type initVolume () const throw ();

      /// \brief Get capsule volume for solution parameters.
      value_type solutionVolume () const throw ();

      /// \brief Get initial capsule parameters.
      const argument_t initParam () const throw ();

      /// \brief Get solution capsule parameters.
      const argument_t solutionParam () const throw ();

      /// \brief Compute best fitting capsule over polyhedron.
      ///
      /// Polyhedron vector attribute is used to compute capsule and set
      /// capsuleParam attribute.
      /// \param initParam initial capsule parameters
      void computeBestFitCapsule (const argument_t& initParam) throw ();

      /// \brief Compute best fitting capsule over polyhedron vector.
      ///
      /// \param polyhedron Polyhedron over which the capsule is
      /// fitted
      /// \param initParam initial capsule parameters
      void computeBestFitCapsule (const polyhedrons_t& polyhedrons,
				  const argument_t& initParam) throw ();

      /// \brief Compute best fitting capsule over polyhedron vector.
      ///
      /// Polyhedron vector attribute is used to compute capsule and set
      /// capsuleParam attribute.
      ///
      /// \param initParam initial capsule parameters
      /// \return capsule parameters
      const argument_t computeBestFitCapsuleParam (const argument_t&
						   initParam) throw ();

      /// \brief Compute best fitting capsule over polyhedron.
      ///
      /// \param polyhedrons Polyhedron vector over which the capsule is
      /// fitted
      ///
      /// \param initParam initial capsule parameters
      /// \return capsule parameters
      const argument_t computeBestFitCapsuleParam (const polyhedrons_t&
						   polyhedrons,
						   const argument_t&
						   initParam) throw ();

    protected:
      /// \brief Implementation of best fitting capsule computation.
      /// \param polyhedrons Polyhedron vector over which the capsule is
      /// fitted
      /// \param initParam initial capsule parameters
      /// \return solutionParam solution capsule parameters
      void impl_computeBestFitCapsuleParam (const polyhedrons_t& polyhedrons,
					    const argument_t& initParam,
					    argument_t& solutionParam) throw ();

    private:
      /// \brief Polyhedron vector attribute.
      polyhedrons_t polyhedrons_;

      /// \brief Initial volume attribute.
      value_type initVolume_;

      /// \brief Solution volume attribute.
      value_type solutionVolume_;

      /// \brief Capsule inital parameters attribute,
      argument_t initParam_;

      /// \brief Capsule solution parameters attribute.
      argument_t solutionParam_;

      /// \brief Nonlinear solver.
      std::string solver_;
    };

    /// \brief Print fitter after optimal capsule has been computed.
    ///
    /// Initial parameters and volume are printed, then solution
    /// parameters and volume are printed.
    inline std::ostream& operator<< (std::ostream& os, const Fitter& fitter)
    {
      using namespace roboptim;
      using roboptim::operator <<;

      os << "Capsule parameters:" << incindent;
      os << iendl << "Initial parameters: " << fitter.initParam ();
      os << iendl << "Initial volume: " << fitter.initVolume ();
      os << iendl << "Solution parameters: " << fitter.solutionParam ();
      os << iendl << "Solution volume: " << fitter.solutionVolume ();
      os << decendl;

      return os;
    }

  } // end of namespace capsule.
} // end of namespace roboptim.

#endif //! ROBOPTIM_CAPSULE_FITTER_HH
