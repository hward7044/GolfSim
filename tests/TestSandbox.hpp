#pragma once
// Every test that touches the filesystem writes under one sandbox root that
// CMake bakes in as an absolute path, so behaviour is identical whether the
// runner is launched by ctest, from the repo root, or from an IDE.
#include <filesystem>
#include <string>

#ifndef GOLFSIM_TEST_SANDBOX_DIR
#error "GOLFSIM_TEST_SANDBOX_DIR must be defined by CMake for the test target"
#endif

namespace TestSandbox {

inline std::filesystem::path root() {
    std::filesystem::path r(GOLFSIM_TEST_SANDBOX_DIR);
    std::filesystem::create_directories(r);
    return r;
}

inline std::string path(const std::string& leaf) {
    return (root() / leaf).string();
}

} // namespace TestSandbox
