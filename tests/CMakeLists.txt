set(TARGET testlib)

set(SOURCE testlib.cpp)

# Strict warnings for engine + test code (REFACTORING.md, Phase 0).
# Turn VPE_WARNINGS_AS_ERRORS on once the baseline is warning-clean.
option(VPE_WARNINGS_AS_ERRORS "Treat warnings as errors in tests" OFF)
if (MSVC)
  set(VPE_WARNING_FLAGS /W4)
  if (VPE_WARNINGS_AS_ERRORS)
    list(APPEND VPE_WARNING_FLAGS /WX)
  endif ()
else ()
  set(VPE_WARNING_FLAGS -Wall -Wextra)
  if (VPE_WARNINGS_AS_ERRORS)
    list(APPEND VPE_WARNING_FLAGS -Werror)
  endif ()
endif ()

add_executable(${TARGET} ${SOURCE})

target_compile_options(${TARGET} PRIVATE ${VPE_WARNING_FLAGS})

target_link_libraries(${TARGET} PRIVATE ViennaPhysicsEngine)

add_test(NAME ${TARGET} COMMAND testlib) # Command can be a target

# Golden-run regression tests (see REFACTORING.md, Phase 0).
# Compare against committed golden files; regenerate only for intended
# behavior changes:  goldenrun --generate ${CMAKE_CURRENT_SOURCE_DIR}/golden
set(VPE_GOLDEN_EPS "0" CACHE STRING "Max per-component deviation for golden-run comparison (0 = exact)")
add_executable(goldenrun goldenrun.cpp)
target_compile_options(goldenrun PRIVATE ${VPE_WARNING_FLAGS})
target_link_libraries(goldenrun PRIVATE ViennaPhysicsEngine)

# Contact pairs are keyed by body addresses (void*), so ASLR changes
# unordered_map iteration order and thus impulse order and float results.
# On Linux, run with ASLR disabled (setarch -R) for bit-exact reproducibility.
# The committed golden files are for the CI reference environment
# (Linux, gcc, Release); on other platforms regenerate them locally first:
#   goldenrun --generate <source>/tests/golden
find_program(VPE_SETARCH setarch)
if (CMAKE_SYSTEM_NAME STREQUAL "Linux" AND VPE_SETARCH)
  add_test(NAME goldenrun COMMAND ${VPE_SETARCH} ${CMAKE_HOST_SYSTEM_PROCESSOR} -R
    $<TARGET_FILE:goldenrun> --compare ${CMAKE_CURRENT_SOURCE_DIR}/golden ${VPE_GOLDEN_EPS})
else ()
  add_test(NAME goldenrun COMMAND goldenrun --compare ${CMAKE_CURRENT_SOURCE_DIR}/golden ${VPE_GOLDEN_EPS})
endif ()
