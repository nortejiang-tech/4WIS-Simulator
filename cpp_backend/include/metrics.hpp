#pragma once
#include "storage.hpp"
namespace fourwis {
double expression(const std::string &source, const Run &run,
                  bool validate_only = false);
json procedure_metrics(const Run &run, const std::string &procedure);
json collect_metrics(const json &requested, const json &base, const Run &run);
json selectors(const std::string &source);
bool matches(const json &coords, const json &filters);
bool holds(double value, const std::string &must);
json criteria(const json &specs, const json &rows, const json &axes);
json comparison(const json &rows, const json &names, const json &axes,
                const json &spec);
} // namespace fourwis
