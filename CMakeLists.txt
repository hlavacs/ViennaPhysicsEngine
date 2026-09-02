cmake_minimum_required(VERSION 3.28)

option(VPE_BUILD_EXAMPLES "Build the VVE-rendered physics example" ON)
option(VPE_BUILD_TESTS "Build standalone VPE tests" OFF)
option(VPE_FORCE_LEFT_HANDED "Build VPE for a left-handed, Y-up coordinate system" OFF)
option(VPE_DOUBLE_ACCURACY "Build VPE with double-precision physics math" OFF)

# How the example consumes ViennaVulkanEngine V3:
#  * in-tree (default on Windows): ../ViennaVulkanEngine is added as a
#    subdirectory and built together with VPE (Ninja + MSVC + `import std`,
#    exactly like VVE's own build_windows.cmd). Binaries land in
#    ../ViennaVulkanEngine/bin/<config>/exe next to the VVE runtime DLLs.
#  * prebuilt (default on Linux/macOS): the example links VVE's already-built
#    release tree and reuses its Clang .pcm module files.
if(WIN32)
  set(VPE_VVE_IN_TREE_DEFAULT ON)
else()
  set(VPE_VVE_IN_TREE_DEFAULT OFF)
endif()
option(VPE_VVE_IN_TREE "Build ../ViennaVulkanEngine as a subdirectory instead of consuming its prebuilt Clang module tree" ${VPE_VVE_IN_TREE_DEFAULT})
option(VPE_BUILD_VVE_EXAMPLES "Also build VVE's own example programs when VVE is built in-tree" OFF)
option(VPE_BUILD_VVE_TESTS "Also build VVE's tests when VVE is built in-tree" OFF)

set(VPE_VVE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../ViennaVulkanEngine")
if(NOT EXISTS "${VPE_VVE_ROOT}/src/Engine.ixx"
   AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/../../ViennaVulkanEngine/src/Engine.ixx")
  # Controller worktrees live one directory below the normal repository layout.
  set(VPE_VVE_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../ViennaVulkanEngine")
endif()
get_filename_component(VPE_VVE_ROOT "${VPE_VVE_ROOT}" ABSOLUTE)

if(VPE_BUILD_EXAMPLES AND VPE_VVE_IN_TREE)
  if(NOT EXISTS "${VPE_VVE_ROOT}/CMakeLists.txt")
    message(FATAL_ERROR "VPE_VVE_IN_TREE is ON but ViennaVulkanEngine was not found at ${VPE_VVE_ROOT}. "
                        "Clone it next to ViennaPhysicsEngine or set VPE_BUILD_EXAMPLES=OFF.")
  endif()
  # VVE requires `import std`. CMake discovers the compiler-provided std module
  # while enabling CXX, so the experimental gate must be set before project().
  # Keep this table in sync with ViennaVulkanEngine/CMakeLists.txt.
  if(CMAKE_VERSION VERSION_LESS "3.31.8")
    message(FATAL_ERROR "Building ViennaVulkanEngine in-tree needs CMake >= 3.31.8 (found ${CMAKE_VERSION}).")
  endif()
  if(CMAKE_VERSION VERSION_LESS "4.0")
    set(VPE_CMAKE_IMPORT_STD_UUID "d0edc3af-4c50-42ea-a356-e2862fe7a444")
  elseif(CMAKE_VERSION VERSION_LESS "4.0.3")
    set(VPE_CMAKE_IMPORT_STD_UUID "a9e1cf81-9932-4810-974b-6eccaf14e457")
  elseif(CMAKE_VERSION VERSION_LESS "4.3")
    set(VPE_CMAKE_IMPORT_STD_UUID "d0edc3af-4c50-42ea-a356-e2862fe7a444")
  elseif(CMAKE_VERSION VERSION_LESS "4.4")
    set(VPE_CMAKE_IMPORT_STD_UUID "451f2fe2-a8a2-47c3-bc32-94786d8fc91b")
  else()
    set(VPE_CMAKE_IMPORT_STD_UUID "f35a9ac6-8463-4d38-8eec-5d6008153e7d")
  endif()
  set(CMAKE_EXPERIMENTAL_CXX_IMPORT_STD "${VPE_CMAKE_IMPORT_STD_UUID}" CACHE STRING "CMake experimental import std gate" FORCE)
  set(CMAKE_CXX_MODULE_STD ON CACHE BOOL "Enable C++ standard library module support" FORCE)
  if(CMAKE_GENERATOR MATCHES "^Visual Studio")
    message(FATAL_ERROR "The Visual Studio generator does not support `import std`, which VVE requires. "
                        "Configure with -G Ninja (see build_cmake.cmd).")
  endif()
endif()

# VVE V3's public facade is a Clang C++23 module. Use the same compiler for VPE
# so its module and the prebuilt VVE module interfaces are ABI compatible.
if(UNIX AND NOT VPE_VVE_IN_TREE AND EXISTS "/usr/bin/clang++-18")
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
get_property(VPE_IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
if(VPE_IS_MULTI_CONFIG)
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release")
elseif(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

if(MSVC)
  add_compile_options(/MP)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wno-reserved-module-identifier)
endif()

if(VPE_BUILD_EXAMPLES AND VPE_VVE_IN_TREE)
  set(BUILD_TESTING ${VPE_BUILD_VVE_TESTS})
  set(VVE_BUILD_EXAMPLES ${VPE_BUILD_VVE_EXAMPLES})
  if(NOT DEFINED VVE_DEFAULT_VULKAN_ICD)
    set(VVE_DEFAULT_VULKAN_ICD "system" CACHE STRING "Default Vulkan ICD selector used when VVE_VULKAN_ICD is unset")
  endif()
  add_subdirectory("${VPE_VVE_ROOT}" "${CMAKE_BINARY_DIR}/ViennaVulkanEngine")
  # Imported targets created by find_package() are visible only in the
  # directory that created them (and below). ViennaVulkanEngine's link
  # interface names assimp::assimp, glm::glm, slang::slang, imgui::imgui,
  # SDL3::SDL3 and Vulkan::Vulkan, so anything outside VVE's directory that
  # links VVE must find the same packages again. Mirror VVE's search paths
  # (its vcpkg tree first, then the Vulkan SDK) so the identical config
  # files are picked up; VVE has already fixed Vulkan_*/SDL3_DIR in the cache.
  list(PREPEND CMAKE_PREFIX_PATH "${VPE_VVE_ROOT}/vcpkg_installed/${VVE_VCPKG_TRIPLET}")
  if(DEFINED VVE_VULKAN_SDK_ROOT AND NOT VVE_VULKAN_SDK_ROOT STREQUAL "")
    list(APPEND CMAKE_PREFIX_PATH "${VVE_VULKAN_SDK_ROOT}" "${VVE_VULKAN_SDK_ROOT}/cmake")
  endif()
  find_package(Vulkan REQUIRED)
  find_package(SDL3 CONFIG REQUIRED)
  find_package(assimp CONFIG REQUIRED)
  find_package(glm CONFIG REQUIRED)
  find_package(imgui CONFIG REQUIRED)
  if(NOT TARGET slang::slang)
    # VVE either found a slang CMake package (directory-scoped, find it again)
    # or created a GLOBAL imported fallback target from the Vulkan SDK.
    find_package(slang CONFIG QUIET)
  endif()
  if(NOT TARGET slang::slang)
    message(FATAL_ERROR "slang::slang is not visible to ViennaPhysicsEngine; VVE's configuration did not provide it.")
  endif()
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

# Windows: STATIC avoids C++20-module symbol export issues with the MSVC
# toolchain (same choice as ViennaVulkanEngine). Linux/macOS: SHARED.
if(WIN32)
  add_library(ViennaPhysicsEngine STATIC)
else()
  add_library(ViennaPhysicsEngine SHARED)
endif()
add_library(ViennaPhysicsEngine::ViennaPhysicsEngine ALIAS ViennaPhysicsEngine)
target_compile_features(ViennaPhysicsEngine PUBLIC cxx_std_23)
target_sources(ViennaPhysicsEngine
    PUBLIC
        FILE_SET CXX_MODULES FILES
            modules/VPE.ixx
    PRIVATE
        modules/VPE.cpp
)
if(TARGET glm::glm)
  # Same glm as ViennaVulkanEngine (vcpkg), so VEEngine and VEPhysicsEngine
  # see identical glm declarations in their global module fragments.
  target_link_libraries(ViennaPhysicsEngine PUBLIC glm::glm)
  if(DEFINED VVE_VULKAN_SDK_ROOT AND NOT VVE_VULKAN_SDK_ROOT STREQUAL ""
     AND EXISTS "${VVE_VULKAN_SDK_ROOT}/include/glm/glm.hpp")
    # VVE puts the Vulkan SDK include dir first; mirror that so both agree.
    target_include_directories(ViennaPhysicsEngine BEFORE PUBLIC "${VVE_VULKAN_SDK_ROOT}/include")
  endif()
else()
  target_include_directories(ViennaPhysicsEngine PUBLIC
      $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/extern>
      $<INSTALL_INTERFACE:include>
  )
endif()
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
get_target_property(VPE_LIB_TYPE ViennaPhysicsEngine TYPE)
if(VPE_LIB_TYPE STREQUAL "SHARED_LIBRARY")
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
  set(BUILD_TESTING ON)
  include(CTest)
  enable_testing()
  add_subdirectory(tests)
endif()
