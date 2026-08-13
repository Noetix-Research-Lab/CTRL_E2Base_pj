#include "rl_controllers/OnnxPolicyModel.h"

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

namespace
{
long currentRssKb()
{
  std::ifstream statm("/proc/self/statm");
  long totalPages = 0;
  long residentPages = 0;
  if (!(statm >> totalPages >> residentPages)) return -1;
  (void)totalPages;
  return residentPages * sysconf(_SC_PAGESIZE) / 1024;
}
}  // namespace

int main(int argc, char** argv)
{
  if (argc < 2 || argc > 3) {
    std::cerr << "usage: model_lifecycle_stress MODEL [ITERATIONS]\n";
    return 2;
  }
  const int iterations = argc == 3 ? std::max(1, std::atoi(argv[2])) : 500;
  try {
    Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "model_lifecycle_stress");
    Ort::SessionOptions options;
    options.SetInterOpNumThreads(1);
    options.SetIntraOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    legged::OnnxPolicyModel model;
    // Warm allocator and shared-library caches before measuring retained RSS.
    for (int index = 0; index < 20; ++index) {
      model.load(environment, argv[1], options);
      model.reset();
    }
    const long startRssKb = currentRssKb();
    for (int index = 0; index < iterations; ++index) {
      model.load(environment, argv[1], options);
      if (!model.loaded() || model.inputNames().empty() || model.outputNames().empty()) return 1;
      model.reset();
    }
    const long endRssKb = currentRssKb();
    std::cout << "{\"iterations\":" << iterations
              << ",\"rss_start_kb\":" << startRssKb
              << ",\"rss_end_kb\":" << endRssKb
              << ",\"rss_growth_kb\":" << endRssKb - startRssKb << "}\n";
  } catch (const std::exception& exception) {
    std::cerr << "lifecycle stress failed: " << exception.what() << '\n';
    return 1;
  }
  return 0;
}
