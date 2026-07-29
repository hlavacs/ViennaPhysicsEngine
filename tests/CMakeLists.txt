#project(MyLibTests LANGUAGES CXX)

include(CTest)

set(TARGET vpe_compile_check)

set(SOURCE vpe_compile_check.cpp)

set(HEADERS )

add_executable(${TARGET} ${SOURCE} ${HEADERS})

target_compile_features(${TARGET} PUBLIC cxx_std_23)

target_link_libraries(${TARGET} PRIVATE ViennaPhysicsEngine::ViennaPhysicsEngine)

add_test(NAME ${TARGET} COMMAND ${TARGET})
