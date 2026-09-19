#pragma once
// Release-safe test assertions. Never compiled away by -DNDEBUG because nothing
// here references NDEBUG or <cassert>; a failure throws, and the runner catches
// per test case.
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>
#include <spdlog/spdlog.h>

struct TestFailure : std::runtime_error {
    using std::runtime_error::runtime_error;
};

#define TEST_ASSERT_MSG(cond, msg)                                                          \
    do {                                                                                    \
        if (!(cond)) {                                                                      \
            spdlog::error("[TEST FAILURE] {}:{}  '{}'  ({})", __FILE__, __LINE__, #cond,    \
                          std::string(msg));                                                \
            throw TestFailure(std::string(#cond) + " @ " + __FILE__ + ":" +                 \
                              std::to_string(__LINE__));                                    \
        }                                                                                   \
    } while (false)

#define TEST_ASSERT(cond) TEST_ASSERT_MSG(cond, "")

// For `std::abs(a - b) < tol` checks: reports the actual values on failure.
#define TEST_NEAR(a, b, tol)                                                                \
    do {                                                                                    \
        const double _a = static_cast<double>(a);                                           \
        const double _b = static_cast<double>(b);                                           \
        const double _t = static_cast<double>(tol);                                         \
        if (!(std::abs(_a - _b) < _t)) {                                                    \
            std::ostringstream _os;                                                         \
            _os << #a " = " << _a << ", " #b " = " << _b << ", tol " << _t;                 \
            TEST_ASSERT_MSG(false, _os.str());                                              \
        }                                                                                   \
    } while (false)
