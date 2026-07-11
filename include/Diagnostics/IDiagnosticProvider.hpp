#pragma once
#include <nlohmann/json.hpp>

class IDiagnosticProvider {
public:
    virtual ~IDiagnosticProvider() = default;
    virtual nlohmann::json getLatestDiagnostics() const = 0;
};
