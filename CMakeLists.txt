# Requires at least CMake 2.6 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/kineo.cmake)

SET(PROJECT_NAME roboptim-capsule)
SET(PROJECT_VERSION 0.1)
SET(PROJECT_DESCRIPTION
  "Utility package for computing best fit capsule over a polygon soup.")
SET(PROJECT_URL "")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(${PROJECT_NAME}_HEADERS
  include/roboptim/capsule/types.hh
  include/roboptim/capsule/volume.hh
  include/roboptim/capsule/distance-capsule-polyhedron.hh
  include/roboptim/capsule/distance-segment-polyhedron.hh
  include/roboptim/capsule/distance-capsule-point.hh
  include/roboptim/capsule/fitter.hh
  )

SETUP_PROJECT()

# Declare dependencies
SEARCH_FOR_BOOST()
ADD_REQUIRED_DEPENDENCY("kcd-capsule")
ADD_REQUIRED_DEPENDENCY("roboptim-core")
ADD_REQUIRED_DEPENDENCY("roboptim-core-plugin-cfsqp")

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
