cmake_minimum_required (VERSION 3.10.0)
project (
  "ViennaPhysicsEngine"
  VERSION 1.0
  DESCRIPTION "A C++20 physics engine for video games."
  LANGUAGES CXX
)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

# enforce C++20 standard
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CONFIGURATION_TYPES "Debug;Release")

option(VPE_BUILD_EXAMPLES "Build the VVE example (needs Vienna Vulkan Engine + Vulkan SDK)" OFF)
option(VPE_BUILD_EXAMPLE_SIMPLE "Build the portable raylib example" OFF)
option(VPE_BUILD_TESTS "Build VPE tests" ON)

# enable clang-tidy. disable to speed up compilation
#set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
#set(CMAKE_CXX_CLANG_TIDY clang-tidy -checks=readability-*;cppcoreguidelines-*;-header-filter=.*; -p build)

add_library(ViennaPhysicsEngine INTERFACE)
target_include_directories(ViennaPhysicsEngine INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern>
  $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
)
target_compile_features(ViennaPhysicsEngine INTERFACE cxx_std_20)

if (VPE_BUILD_EXAMPLES OR VPE_BUILD_EXAMPLE_SIMPLE)
  add_subdirectory(examples)
endif ()

install(TARGETS ViennaPhysicsEngine
  EXPORT ViennaPhysicsEngineTargets
)
install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(DIRECTORY extern/glm DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(EXPORT ViennaPhysicsEngineTargets
  NAMESPACE ViennaPhysicsEngine::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ViennaPhysicsEngine
)
configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/ViennaPhysicsEngineConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/ViennaPhysicsEngineConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ViennaPhysicsEngine
)
write_basic_package_version_file(
  ${CMAKE_CURRENT_BINARY_DIR}/ViennaPhysicsEngineConfigVersion.cmake
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
install(FILES
  ${CMAKE_CURRENT_BINARY_DIR}/ViennaPhysicsEngineConfig.cmake
  ${CMAKE_CURRENT_BINARY_DIR}/ViennaPhysicsEngineConfigVersion.cmake
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/ViennaPhysicsEngine
)

if (${CMAKE_SOURCE_DIR} STREQUAL ${CMAKE_CURRENT_SOURCE_DIR})
  if (VPE_BUILD_TESTS)
    include(CTest)
    add_subdirectory(tests)
  endif ()
endif ()
