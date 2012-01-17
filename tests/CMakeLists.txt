# Add Boost path to include directories.
INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIRS})

# Add src to link directories.
LINK_DIRECTORIES(../src)

# Make Boost.Test generates the main function in test cases.
ADD_DEFINITIONS(-DBOOST_TEST_DYN_LINK -DBOOST_TEST_MAIN)

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc', link it against
# Boost and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME)
  ADD_EXECUTABLE(${NAME} ${CMAKE_CURRENT_SOURCE_DIR}/${NAME}.cc)
  ADD_TEST(${NAME} ${RUNTIME_OUTPUT_DIRECTORY}/${NAME})

  PKG_CONFIG_USE_DEPENDENCY(${NAME} hpp-geometry)
  PKG_CONFIG_USE_DEPENDENCY(${NAME} roboptim-core)

  # Link against package library.
  TARGET_LINK_LIBRARIES(${NAME} 
    ${Boost_LIBRARIES}
    ${PROJECT_NAME})
ENDMACRO(ADD_TESTCASE)

# Generated test.
ADD_TESTCASE(capsule-volume)
KINEO_STANDALONE(capsule-volume)

ADD_TESTCASE(distance-capsule-polyhedron)
KINEO_STANDALONE(distance-capsule-polyhedron)

ADD_TESTCASE(distance-segment-polyhedron)
KINEO_STANDALONE(distance-segment-polyhedron)

ADD_TESTCASE(distance-capsule-point)
KINEO_STANDALONE(distance-capsule-point)

ADD_TESTCASE(fitter)
KINEO_STANDALONE(fitter)
