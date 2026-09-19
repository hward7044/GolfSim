// GolfSimTests — standalone test runner.
//
//   GolfSimTests                     run every registered case
//   GolfSimTests Units Kinematics    run only the named cases (exit 2 if a name is unknown)
//   GolfSimTests --filter Recorder   run cases whose name contains the substring
//   GolfSimTests --list              print case names and exit
//   GolfSimTests --verbose           show spdlog info-level output from the code under test
//
// Exit codes: 0 all passed, 1 at least one failure, 2 bad arguments / unknown case.

#include "TestRegistry.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <iostream>
#include <string>
#include <vector>
#include <spdlog/spdlog.h>

namespace {

void printUsage() {
    std::cout << "Usage: GolfSimTests [--list] [--verbose] [--filter <substr>] [<case>...]\n";
}

} // namespace

int main(int argc, char* argv[]) {
    bool listOnly = false;
    bool verbose = false;
    std::string filter;
    std::vector<std::string> requested;

    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg == "--list") {
            listOnly = true;
        } else if (arg == "--verbose" || arg == "-v") {
            verbose = true;
        } else if (arg == "--filter" && i + 1 < argc) {
            filter = argv[++i];
        } else if (arg == "--help" || arg == "-h") {
            printUsage();
            return 0;
        } else if (!arg.empty() && arg[0] == '-') {
            std::cerr << "Unknown option: " << arg << "\n";
            printUsage();
            return 2;
        } else {
            requested.push_back(arg);
        }
    }

    auto& registry = testRegistry();

    if (listOnly) {
        for (const auto& tc : registry) {
            std::cout << tc.name << "\n";
        }
        return 0;
    }

    // Resolve the set of cases to run
    std::vector<const TestCase*> selected;
    if (!requested.empty()) {
        for (const auto& name : requested) {
            auto it = std::find_if(registry.begin(), registry.end(),
                                   [&](const TestCase& tc) { return tc.name == name; });
            if (it == registry.end()) {
                std::cerr << "Unknown test case: " << name << "\n";
                return 2;
            }
            selected.push_back(&*it);
        }
    } else {
        for (const auto& tc : registry) {
            if (filter.empty() || tc.name.find(filter) != std::string::npos) {
                selected.push_back(&tc);
            }
        }
    }

    if (selected.empty()) {
        std::cerr << "No test cases selected.\n";
        return 2;
    }

    // Quiet by default so [TEST] chatter from the code under test does not
    // drown the PASS/FAIL lines; failures are always logged at error level.
    spdlog::set_level(verbose ? spdlog::level::info : spdlog::level::warn);

    int failures = 0;
    auto suiteStart = std::chrono::steady_clock::now();

    for (const TestCase* tc : selected) {
        auto start = std::chrono::steady_clock::now();
        try {
            tc->fn();
            auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                          std::chrono::steady_clock::now() - start).count();
            std::cout << "PASS " << tc->name << " (" << ms << " ms)\n";
        } catch (const std::exception& e) {
            ++failures;
            std::cout << "FAIL " << tc->name << ": " << e.what() << "\n";
        } catch (...) {
            ++failures;
            std::cout << "FAIL " << tc->name << ": non-standard exception\n";
        }
    }

    auto totalMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - suiteStart).count();
    std::cout << "\n" << (selected.size() - failures) << "/" << selected.size()
              << " passed in " << totalMs << " ms\n";

    return failures == 0 ? 0 : 1;
}
