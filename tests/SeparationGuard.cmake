# Guards that keep test code out of the production tree and binary.
# Included from the top-level CMakeLists.txt when GOLFSIM_BUILD_TESTS is ON.

# 1. No test identifiers anywhere under src/ or include/.
#    The pattern is deliberately broad: a comment mentioning a mock type fails too.
add_test(
    NAME NoTestCodeInProduction
    COMMAND ${CMAKE_COMMAND}
        -DSOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR}
        -P ${CMAKE_CURRENT_SOURCE_DIR}/tests/NoTestCodeInProduction.cmake
)

# 2. No test-registry symbols linked into the production executable (Linux only;
#    the grep guard above covers Windows).
find_program(GOLFSIM_NM nm)
if(GOLFSIM_NM AND NOT WIN32)
    add_test(
        NAME NoTestSymbolsInBinary
        COMMAND sh -c "! ${GOLFSIM_NM} -C $<TARGET_FILE:GolfSim> | grep -E 'TestRegistrar|[^A-Za-z_]test_[A-Z]'"
    )
endif()
