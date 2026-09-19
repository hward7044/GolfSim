# Merge raw profiles and produce the llvm-cov summary, HTML, JSON and LCOV outputs.
# Invoked by the `coverage` custom target; see docs/refactor/07_Test_Infrastructure_Plan.md §8.4.
#
# Inputs (-D): PROFRAW_DIR, LLVM_PROFDATA, LLVM_COV, BINARY, OUT_DIR, SOURCE_DIR

file(GLOB PROFRAW_FILES "${PROFRAW_DIR}/*.profraw")
if(NOT PROFRAW_FILES)
    message(FATAL_ERROR "No .profraw files in ${PROFRAW_DIR}; did the tests run under LLVM_PROFILE_FILE?")
endif()

set(PROFDATA "${OUT_DIR}/merged.profdata")
execute_process(
    COMMAND ${LLVM_PROFDATA} merge -sparse -o ${PROFDATA} ${PROFRAW_FILES}
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "llvm-profdata merge failed (${rc})")
endif()

# Library headers and the test tree are never part of the measured scope.
set(IGNORE_RE "(/tests/|/usr/|/\\.local/)")
set(COMMON ${BINARY} -instr-profile=${PROFDATA} -ignore-filename-regex=${IGNORE_RE})

# 1. Human-readable summary (also saved next to the JSON)
execute_process(
    COMMAND ${LLVM_COV} report ${COMMON} -show-branch-summary -show-mcdc-summary
    OUTPUT_FILE ${OUT_DIR}/coverage-summary.txt
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "llvm-cov report failed (${rc})")
endif()
file(READ ${OUT_DIR}/coverage-summary.txt SUMMARY)
message(STATUS "\n${SUMMARY}")

# 2. Browsable HTML
execute_process(
    COMMAND ${LLVM_COV} show ${COMMON} -format=html -output-dir=${OUT_DIR}/coverage-html
            -show-branches=count -show-mcdc -show-line-counts-or-regions -Xdemangler c++filt
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(WARNING "llvm-cov show (html) failed (${rc}); continuing")
endif()

# 3. Machine-readable outputs for the threshold gate
execute_process(
    COMMAND ${LLVM_COV} export ${COMMON} -format=text -summary-only
    OUTPUT_FILE ${OUT_DIR}/coverage.json
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "llvm-cov export (json) failed (${rc})")
endif()
execute_process(
    COMMAND ${LLVM_COV} export ${COMMON} -format=lcov
    OUTPUT_FILE ${OUT_DIR}/coverage.lcov
    RESULT_VARIABLE rc)
if(NOT rc EQUAL 0)
    message(FATAL_ERROR "llvm-cov export (lcov) failed (${rc})")
endif()

message(STATUS "Coverage report: ${OUT_DIR}/coverage-html/index.html")
