set(TARGET testlib)

set(SOURCE testlib.cpp)

add_executable(${TARGET} ${SOURCE})

target_link_libraries(${TARGET} PRIVATE ViennaPhysicsEngine)

add_test(NAME ${TARGET} COMMAND testlib) # Command can be a target
