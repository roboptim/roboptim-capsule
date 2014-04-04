# Requires at least CMake 2.8 to configure the package.
CMAKE_MINIMUM_REQUIRED(VERSION 2.8)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/boost.cmake)
INCLUDE(cmake/qhull.cmake)

SET(PROJECT_NAME roboptim-capsule)
SET(PROJECT_DESCRIPTION
  "An automated optimal capsule generator over polyhedrons.")
SET(PROJECT_URL "https://github.com/roboptim/roboptim-capsule")

# Where to compile shared objects
SET(LIBRARY_OUTPUT_PATH ${PROJECT_BINARY_DIR}/lib)

SET(${PROJECT_NAME}_HEADERS
  include/roboptim/capsule/distance-capsule-point.hh
  include/roboptim/capsule/fwd.hh
  include/roboptim/capsule/fitter.hh
  include/roboptim/capsule/qhull.hh
  include/roboptim/capsule/types.hh
  include/roboptim/capsule/util.hh
  include/roboptim/capsule/volume.hh
  )

SETUP_PROJECT()

# Declare dependencies
SEARCH_FOR_BOOST()
SEARCH_FOR_QHULL()
ADD_REQUIRED_DEPENDENCY("eigen3 >= 3.2.0")
ADD_REQUIRED_DEPENDENCY("roboptim-core")
ADD_REQUIRED_DEPENDENCY("roboptim-core-plugin-ipopt")

# Add main library to pkg-config file.
PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
