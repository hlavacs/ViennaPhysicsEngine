cmake_minimum_required(VERSION 3.28)

# VVE V3's public facade is a Clang C++23 module. Use the same compiler as the
# already-built VVE module interfaces so their compiled module files are ABI
# compatible with this consumer.
if(UNIX AND EXISTS "/usr/bin/clang++-18")
  set(CMAKE_CXX_COMPILER "/usr/bin/clang++-18" CACHE FILEPATH
      "Compiler used by the prebuilt VVE V3 modules" FORCE)
endif()

project (
  "ViennaPhysicsEngine"
  VERSION 1.0
  DESCRIPTION "A C++20 physics engine for video games."
  LANGUAGES CXX
)

# VVE V3 exports the standard-library module in C++23 mode.
set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

# enable clang-tidy. disable to speed up compilation
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=readability-*;cppcoreguidelines-*;-header-filter=.*; -p build)

add_subdirectory(examples)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  #include(CTest)
  #add_subdirectory(tests)
endif ()
