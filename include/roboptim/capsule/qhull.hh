// Copyright (C) 2014 by Benjamin Chretien, CNRS-LIRMM.
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

#ifndef ROBOPTIM_CAPSULE_QHULL_HH_
# define ROBOPTIM_CAPSULE_QHULL_HH_

# ifdef HAVE_QHULL

#  if defined __GNUC__
#   pragma GCC system_header
#  endif

extern "C"
{
#  ifdef HAVE_QHULL_2011
#   include <libqhull/libqhull.h>
#   include <libqhull/mem.h>
#   include <libqhull/qset.h>
#   include <libqhull/geom.h>
#   include <libqhull/merge.h>
#   include <libqhull/poly.h>
#   include <libqhull/io.h>
#   include <libqhull/stat.h>
#  else
#   include <qhull/qhull.h>
#   include <qhull/mem.h>
#   include <qhull/qset.h>
#   include <qhull/geom.h>
#   include <qhull/merge.h>
#   include <qhull/poly.h>
#   include <qhull/io.h>
#   include <qhull/stat.h>
#  endif //! HAVE_QHULL_2011
}

# endif //! HAVE_QHULL

#endif //! ROBOPTIM_CAPSULE_QHULL_HH_
