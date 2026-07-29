add_executable(physicsexample
    physicsexample.cpp
    VPEConstraintDemos.cpp
    VPEConstraintDemos.hpp
    ${CMAKE_SOURCE_DIR}/include/VPE.hpp
)

set(VVE_V3_ROOT "${CMAKE_SOURCE_DIR}/../ViennaVulkanEngine")
if(NOT EXISTS "${VVE_V3_ROOT}/src/Engine.ixx"
   AND EXISTS "${CMAKE_SOURCE_DIR}/../../ViennaVulkanEngine/src/Engine.ixx")
    # Controller worktrees live one directory below the normal repository
    # layout; keep the documented sibling layout as the primary location.
    set(VVE_V3_ROOT "${CMAKE_SOURCE_DIR}/../../ViennaVulkanEngine")
endif()
set(VVE_V3_VCPKG_ROOT "${VVE_V3_ROOT}/vcpkg_installed/x64-linux")
set(VVE_V3_LIBRARY
    "${VVE_V3_ROOT}/bin/release/lib/libViennaVulkanEngine.so")
set(VVE_V3_MODULE_DIR
    "${VVE_V3_ROOT}/build/release-linux/src/CMakeFiles/ViennaVulkanEngine.dir")
set(VVE_V3_STD_MODULE_DIR
    "${VVE_V3_ROOT}/build/release-linux/src/CMakeFiles/__cmake_cxx_std_23.dir")
set(VVE_V3_STD_MODULE_LIBRARY
    "${VVE_V3_ROOT}/build/release-linux/src/lib__cmake_cxx_std_23.a")

# FindVulkan does not discover the loader in VVE's local vcpkg installation
# automatically. Set both cache entries before find_package so the example uses
# the same Vulkan headers and loader as the prebuilt VVE V3 library.
set(Vulkan_LIBRARY
    "${VVE_V3_VCPKG_ROOT}/lib/libvulkan.so"
    CACHE FILEPATH "Vulkan loader from the VVE V3 vcpkg installation" FORCE)
set(Vulkan_INCLUDE_DIR
    "${VVE_V3_VCPKG_ROOT}/include"
    CACHE PATH "Vulkan headers from the VVE V3 vcpkg installation" FORCE)
find_package(Vulkan REQUIRED)

add_library(ViennaVulkanEngineV3 SHARED IMPORTED)
set_target_properties(ViennaVulkanEngineV3 PROPERTIES
    IMPORTED_LOCATION "${VVE_V3_LIBRARY}"
)

add_library(VVEV3ImGui STATIC IMPORTED)
set_target_properties(VVEV3ImGui PROPERTIES
    IMPORTED_LOCATION "${VVE_V3_VCPKG_ROOT}/lib/libimgui.a"
)

add_library(VVEV3StdModule STATIC IMPORTED)
set_target_properties(VVEV3StdModule PROPERTIES
    IMPORTED_LOCATION "${VVE_V3_STD_MODULE_LIBRARY}"
)

set_target_properties(physicsexample PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
    BUILD_RPATH "${VVE_V3_ROOT}/bin/release/lib"
)

target_include_directories(physicsexample PUBLIC
    ${CMAKE_SOURCE_DIR}/include
    ${Vulkan_INCLUDE_DIRS}
    ${VVE_V3_ROOT}/src
    ${VVE_V3_VCPKG_ROOT}/include
)

target_compile_definitions(physicsexample PRIVATE
    VULKAN_HPP_NO_EXCEPTIONS
    VVE_ENGINE_IMPLEMENTATION_NAMESPACE=simple
    VVE_ENGINE_IMPLEMENTATION_IS_SIMPLE
    VVE_ENGINE_HAS_SIMPLE
    VVE_V3_ROOT_PATH="${VVE_V3_ROOT}"
)

# Reuse the compiled public module interfaces from the already-built VVE V3
# release tree. This consumes the V3 facade without rebuilding the engine.
target_compile_options(physicsexample PRIVATE
    -stdlib=libc++
    "-fmodule-file=std=${VVE_V3_STD_MODULE_DIR}/std.pcm"
    "-fmodule-file=VEEngine=${VVE_V3_MODULE_DIR}/VEEngine.pcm"
    "-fmodule-file=VEEngine.ECSContainer=${VVE_V3_MODULE_DIR}/VEEngine.ECSContainer.pcm"
    "-fmodule-file=VEEngine.Entity=${VVE_V3_MODULE_DIR}/VEEngine.Entity.pcm"
    "-fmodule-file=VEEngine.Error=${VVE_V3_MODULE_DIR}/VEEngine.Error.pcm"
    "-fmodule-file=VEEngine.Handle=${VVE_V3_MODULE_DIR}/VEEngine.Handle.pcm"
    "-fmodule-file=VEEngine.Math=${VVE_V3_MODULE_DIR}/VEEngine.Math.pcm"
    "-fmodule-file=VEEngine.Types=${VVE_V3_MODULE_DIR}/VEEngine.Types.pcm"
    "-fmodule-file=VEEngine.Vector=${VVE_V3_MODULE_DIR}/VEEngine.Vector.pcm"
    "-fmodule-file=VEEngine:Assets=${VVE_V3_MODULE_DIR}/VEEngine-Assets.pcm"
    "-fmodule-file=VEEngine:ECS=${VVE_V3_MODULE_DIR}/VEEngine-ECS.pcm"
    "-fmodule-file=VEEngine:Gui=${VVE_V3_MODULE_DIR}/VEEngine-Gui.pcm"
    "-fmodule-file=VEEngine:RenderSystem=${VVE_V3_MODULE_DIR}/VEEngine-RenderSystem.pcm"
    "-fmodule-file=VEEngine:Window=${VVE_V3_MODULE_DIR}/VEEngine-Window.pcm"
    "-fmodule-file=VEEngine:World=${VVE_V3_MODULE_DIR}/VEEngine-World.pcm"
)

target_link_options(physicsexample PRIVATE
    -stdlib=libc++
    -pthread
)

target_link_directories(physicsexample PRIVATE
    ${VVE_V3_VCPKG_ROOT}/lib
)

target_link_libraries(physicsexample PRIVATE
    ViennaVulkanEngineV3
    VVEV3ImGui
    VVEV3StdModule
    Vulkan::Vulkan
)
