#pragma once
// Self-registering test cases. The registry is a function-local static so
// static-initialisation order across translation units does not matter.
#include <functional>
#include <string>
#include <vector>

struct TestCase {
    std::string name;
    std::function<void()> fn;
};

inline std::vector<TestCase>& testRegistry() {
    static std::vector<TestCase> registry;
    return registry;
}

struct TestRegistrar {
    TestRegistrar(const char* name, void (*fn)()) {
        testRegistry().push_back({name, fn});
    }
};

// GOLFSIM_TEST(KinematicsEngine) { ... }   -> registers "KinematicsEngine"
#define GOLFSIM_TEST(name)                                       \
    static void test_##name();                                   \
    static TestRegistrar registrar_##name(#name, &test_##name);  \
    static void test_##name()
