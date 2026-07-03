# Portable VPE example using raylib (fetched automatically; no local install
# needed). Enable with -DVPE_BUILD_EXAMPLE_SIMPLE=ON.

include(FetchContent)

set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)   # raylib's own examples/games
set(BUILD_GAMES OFF CACHE BOOL "" FORCE)

FetchContent_Declare(
  raylib
  GIT_REPOSITORY https://github.com/raysan5/raylib.git
  GIT_TAG 5.5
  GIT_SHALLOW TRUE
)
FetchContent_MakeAvailable(raylib)

add_executable(simpleexample main.cpp)
target_link_libraries(simpleexample PRIVATE ViennaPhysicsEngine raylib)
