cmake_minimum_required(VERSION 3.28)

option(VPE_BUILD_EXAMPLES "Build the VVE-rendered physics example" ON)
option(VPE_BUILD_TESTS "Build standalone VPE tests" OFF)
option(VPE_FORCE_LEFT_HANDED "Build VPE for a left-handed, Y-up coordinate system" OFF)
option(VPE_DOUBLE_ACCURACY "Build VPE with double-precision physics math" OFF)

# VVE V3's public facade is a Clang C++23 module. Use the same compiler for VPE
# so its module and the prebuilt VVE module interfaces are ABI compatible.
if(UNIX AND EXISTS "/usr/bin/clang++-18")
  set(CMAKE_CXX_COMPILER "/usr/bin/clang++-18" CACHE FILEPATH
      "Compiler used by the VPE and VVE V3 modules" FORCE)
endif()

project (
  "ViennaPhysicsEngine"
  VERSION 1.0
  DESCRIPTION "A C++23 physics engine for video games."
  LANGUAGES CXX
)

set(CMAKE_CXX_STANDARD 23)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-reserved-module-identifier)
endif()

# Keep build-tree artifacts isolated per configuration, then mirror public
# runtime artifacts to the source-tree bin layout used by VVE.
set(VPE_OUTPUT_CONFIG_DIR "$<LOWER_CASE:$<CONFIG>>")
set(VPE_BUILD_LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_BINARY_DIR}/bin/${VPE_OUTPUT_CONFIG_DIR}/lib")
set(VPE_PROJECT_LIBRARY_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin/${VPE_OUTPUT_CONFIG_DIR}/lib")
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    "${CMAKE_SOURCE_DIR}/bin/${VPE_OUTPUT_CONFIG_DIR}/exe")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY
    "${VPE_BUILD_LIBRARY_OUTPUT_DIRECTORY}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY
    "${VPE_BUILD_LIBRARY_OUTPUT_DIRECTORY}")

# enable clang-tidy. disable to speed up compilation
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=readability-*;cppcoreguidelines-*;-header-filter=.*; -p build)

add_library(ViennaPhysicsEngine SHARED)
add_library(ViennaPhysicsEngine::ViennaPhysicsEngine ALIAS ViennaPhysicsEngine)
target_compile_features(ViennaPhysicsEngine PUBLIC cxx_std_23)
target_sources(ViennaPhysicsEngine
    PUBLIC
        FILE_SET CXX_MODULES FILES
            modules/VPE.ixx
    PRIVATE
        modules/VPE.cpp
)
target_include_directories(ViennaPhysicsEngine PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern>
    $<INSTALL_INTERFACE:include>
)
target_compile_definitions(ViennaPhysicsEngine PRIVATE
    "$<$<BOOL:${VPE_FORCE_LEFT_HANDED}>:GLM_FORCE_LEFT_HANDED>"
    "$<$<BOOL:${VPE_DOUBLE_ACCURACY}>:VPE_DOUBLE_ACCURACY>"
)
target_compile_options(ViennaPhysicsEngine PUBLIC
    "$<$<CXX_COMPILER_ID:Clang>:-stdlib=libc++>"
)
target_link_options(ViennaPhysicsEngine PUBLIC
    "$<$<CXX_COMPILER_ID:Clang>:-stdlib=libc++>"
)
if(NOT WIN32)
  add_custom_command(TARGET ViennaPhysicsEngine POST_BUILD
      COMMAND "${CMAKE_COMMAND}" -E make_directory
          "${VPE_PROJECT_LIBRARY_OUTPUT_DIRECTORY}"
      COMMAND "${CMAKE_COMMAND}" -E copy_if_different
          "$<TARGET_FILE:ViennaPhysicsEngine>"
          "${VPE_PROJECT_LIBRARY_OUTPUT_DIRECTORY}"
      COMMAND_EXPAND_LISTS
  )
endif()

if(VPE_BUILD_EXAMPLES)
  add_subdirectory(examples)
endif()

if(VPE_BUILD_TESTS)
  include(CTest)
  add_subdirectory(tests)
endif()
