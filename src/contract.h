#pragma once

#include <cstdint>

#include "ch.h"
#include "graph.h"

namespace rr {

struct ContractOptions {
  int threads = 1;
  std::uint32_t settle_limit = 500;
  bool verbose = false;
};

struct ContractReport {
  std::uint32_t rounds = 0;
  std::uint64_t witness_searches = 0;
  std::uint32_t max_degree = 0;
  std::uint64_t peak_entries = 0;
  double seconds = 0;
};

Result<ChGraph> Contract(const Graph& graph, const ContractOptions& options,
                         ContractReport* report);

}
