cmake_minimum_required(VERSION 3.28)

option(VPE_BUILD_EXAMPLES "Build the VVE-rendered physics example" ON)
option(VPE_BUILD_TESTS "Build standalone VPE tests" OFF)

# VVE V3's public facade is a Clang C++23 module. Use the same compiler as the
# already-built VVE module interfaces so their compiled module files are ABI
# compatible with this consumer.
if(VPE_BUILD_EXAMPLES AND UNIX AND EXISTS "/usr/bin/clang++-18")
  set(CMAKE_CXX_COMPILER "/usr/bin/clang++-18" CACHE FILEPATH
      "Compiler used by the prebuilt VVE V3 modules" FORCE)
endif()

project (
  "ViennaPhysicsEngine"
  VERSION 1.0
  DESCRIPTION "A C++20 physics engine for video games."
  LANGUAGES CXX
)

# VPE itself is C++20; only the VVE facade consumer requires C++23.
if(VPE_BUILD_EXAMPLES)
  set(CMAKE_CXX_STANDARD 23)
else()
  set(CMAKE_CXX_STANDARD 20)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

# Keep every VPE executable and library in the same source-tree bin layout as VVE.
set(VPE_OUTPUT_CONFIG_DIR "$<LOWER_CASE:$<CONFIG>>")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin/${VPE_OUTPUT_CONFIG_DIR}/exe")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin/${VPE_OUTPUT_CONFIG_DIR}/lib")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin/${VPE_OUTPUT_CONFIG_DIR}/lib")

# enable clang-tidy. disable to speed up compilation
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=readability-*;cppcoreguidelines-*;-header-filter=.*; -p build)

add_library(ViennaPhysicsEngine INTERFACE)
add_library(ViennaPhysicsEngine::ViennaPhysicsEngine ALIAS ViennaPhysicsEngine)
target_compile_features(ViennaPhysicsEngine INTERFACE cxx_std_20)
target_include_directories(ViennaPhysicsEngine INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern>
    $<INSTALL_INTERFACE:include>
)

if(VPE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(VPE_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()
