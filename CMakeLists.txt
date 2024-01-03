cmake_minimum_required (VERSION 3.5.0)
project (
  "ViennaPhysicsEngine"
  VERSION 1.0
  DESCRIPTION "A C++20 physics engine for video games."
  LANGUAGES CXX
)

# enforce C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

#add_subdirectory(examples)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  include(CTest)
  add_subdirectory(tests)
endif ()
