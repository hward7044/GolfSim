# Fails if any file under src/ or include/ references a test-only identifier.
file(GLOB_RECURSE PRODUCTION_FILES
    "${SOURCE_DIR}/src/*.cpp" "${SOURCE_DIR}/src/*.hpp"
    "${SOURCE_DIR}/include/*.hpp" "${SOURCE_DIR}/include/*.h")

set(PATTERN "(TEST_ASSERT|GOLFSIM_TEST|TestRegistrar|TestSandbox|Mock[A-Z][A-Za-z]*|runMathTests)")
set(VIOLATIONS "")

foreach(f ${PRODUCTION_FILES})
    file(STRINGS "${f}" LINES REGEX "${PATTERN}")
    foreach(line ${LINES})
        list(APPEND VIOLATIONS "${f}: ${line}")
    endforeach()
endforeach()

if(VIOLATIONS)
    list(JOIN VIOLATIONS "\n  " MSG)
    message(FATAL_ERROR "Test code found in production sources:\n  ${MSG}")
endif()
message(STATUS "No test identifiers found in src/ or include/.")
