

add_executable( physicsexample main.cpp VPEConstraintDemos.cpp VPEConstraintDemos.hpp)

set( VulkanEngine ${CMAKE_SOURCE_DIR}/../ViennaVulkanEngine )

set_target_properties(physicsexample PROPERTIES
            RUNTIME_OUTPUT_DIRECTORY ${VulkanEngine}/bin
            VS_DEBUGGER_WORKING_DIRECTORY ${VulkanEngine}/bin/Debug
            )

find_package(Vulkan REQUIRED)

include_directories(
      ${CMAKE_SOURCE_DIR}
      ${Vulkan_INCLUDE_DIRS}
      ${VulkanEngine}/VulkanEngine
      ${VulkanEngine}/external/Assimp/include
      ${VulkanEngine}/external/irrKlang/include
      ${VulkanEngine}/external/glm
      ${VulkanEngine}/external/threadpool
      ${VulkanEngine}/external/stb
      ${VulkanEngine}/external/nuklear
      ${VulkanEngine}/external/glfw/include
      )

set( AssimpDir ${VulkanEngine}/external/Assimp/lib/x64)
set( AssimpLibRel ${AssimpDir}/Release/assimp-vc140-mt.lib ${AssimpDir}/Release/IrrXML.lib ${AssimpDir}/Release/zlib.lib )
set( AssimpLibDeb ${AssimpDir}/Debug/assimp-vc140-mt.lib ${AssimpDir}/Debug/IrrXML.lib ${AssimpDir}/Debug/zlibd.lib )
add_library(AssimpLib STATIC IMPORTED)
set_target_properties(AssimpLib PROPERTIES
       IMPORTED_CONFIGURATIONS "Debug;Release"
       IMPORTED_LOCATION ${AssimpLibRel}
       IMPORTED_LOCATION_DEBUG ${AssimpLibDeb}
       )

add_library(VulkanEngineLib STATIC IMPORTED)
set_target_properties(VulkanEngineLib PROPERTIES
             IMPORTED_CONFIGURATIONS "Debug;Release"
             IMPORTED_LOCATION ${VulkanEngine}/lib/Release/vulkanengine.lib
             IMPORTED_LOCATION_DEBUG ${VulkanEngine}/lib/Debug/vulkanengine.lib
             )

target_link_libraries(physicsexample AssimpLib VulkanEngineLib ${VulkanEngine}/external/glfw/lib-vc2022/glfw3.lib)
