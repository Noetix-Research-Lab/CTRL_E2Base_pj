#include "rl_controllers/OnnxPolicyModel.h"

#include <algorithm>
#include <chrono>
#include <cstdlib>
#include <iomanip>
#include <fstream>
#include <iostream>
#include <numeric>
#include <stdexcept>
#include <string>
#include <sys/resource.h>
#include <unistd.h>
#include <vector>

namespace
{
size_t elementCount(const std::vector<int64_t>& shape)
{
  size_t count = 1;
  for (const int64_t dimension : shape) {
    if (dimension <= 0) throw std::runtime_error("benchmark requires static positive input shapes");
    count *= static_cast<size_t>(dimension);
  }
  return count;
}

double percentile(const std::vector<double>& sorted, double quantile)
{
  if (sorted.empty()) return 0.0;
  const size_t index = static_cast<size_t>(
      std::min<double>(sorted.size() - 1, quantile * static_cast<double>(sorted.size() - 1)));
  return sorted[index];
}

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
  if (argc < 2 || argc > 4) {
    std::cerr << "usage: policy_latency_benchmark MODEL [ITERATIONS] [WARMUP]\n";
    return 2;
  }
  const int iterations = argc >= 3 ? std::max(1, std::atoi(argv[2])) : 1000;
  const int warmup = argc >= 4 ? std::max(0, std::atoi(argv[3])) : 50;

  try {
    Ort::Env environment(ORT_LOGGING_LEVEL_ERROR, "policy_latency_benchmark");
    Ort::SessionOptions options;
    options.SetInterOpNumThreads(1);
    options.SetIntraOpNumThreads(1);
    options.SetExecutionMode(ExecutionMode::ORT_SEQUENTIAL);
    legged::OnnxPolicyModel model;
    model.load(environment, argv[1], options);

    std::vector<std::vector<float>> inputs;
    inputs.reserve(model.inputShapes().size());
    size_t inputElements = 0;
    for (const auto& shape : model.inputShapes()) {
      const size_t size = elementCount(shape);
      inputs.emplace_back(size, 0.0F);
      inputElements += size;
    }
    Ort::MemoryInfo memory =
        Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);
    Ort::RunOptions runOptions;
    auto run = [&]() {
      std::vector<Ort::Value> tensors;
      tensors.reserve(inputs.size());
      for (size_t index = 0; index < inputs.size(); ++index) {
        tensors.emplace_back(Ort::Value::CreateTensor<float>(
            memory, inputs[index].data(), inputs[index].size(),
            model.inputShapes()[index].data(), model.inputShapes()[index].size()));
      }
      return model.session().Run(
          runOptions, model.inputNames().data(), tensors.data(), tensors.size(),
          model.outputNames().data(), model.outputNames().size());
    };
    std::vector<double> latencyUs(static_cast<size_t>(iterations), 0.0);
    for (int index = 0; index < warmup; ++index) (void)run();
    const long startRssKb = currentRssKb();
    const auto wallStart = std::chrono::steady_clock::now();
    for (int index = 0; index < iterations; ++index) {
      const auto start = std::chrono::steady_clock::now();
      const auto output = run();
      if (output.empty()) throw std::runtime_error("model returned no outputs");
      const auto end = std::chrono::steady_clock::now();
      latencyUs[static_cast<size_t>(index)] =
          std::chrono::duration<double, std::micro>(end - start).count();
    }
    const auto wallEnd = std::chrono::steady_clock::now();
    std::sort(latencyUs.begin(), latencyUs.end());
    const double mean =
        std::accumulate(latencyUs.begin(), latencyUs.end(), 0.0) / latencyUs.size();
    const double wallSeconds = std::chrono::duration<double>(wallEnd - wallStart).count();
    rusage usage{};
    getrusage(RUSAGE_SELF, &usage);
    const long endRssKb = currentRssKb();

    std::cout << std::fixed << std::setprecision(3)
              << "{\"iterations\":" << iterations
              << ",\"input_count\":" << inputs.size()
              << ",\"input_elements\":" << inputElements
              << ",\"mean_us\":" << mean
              << ",\"p50_us\":" << percentile(latencyUs, 0.50)
              << ",\"p95_us\":" << percentile(latencyUs, 0.95)
              << ",\"p99_us\":" << percentile(latencyUs, 0.99)
              << ",\"max_us\":" << latencyUs.back()
              << ",\"throughput_hz\":" << iterations / wallSeconds
              << ",\"rss_start_kb\":" << startRssKb
              << ",\"rss_end_kb\":" << endRssKb
              << ",\"rss_growth_kb\":" << (endRssKb - startRssKb)
              << ",\"max_rss_kb\":" << usage.ru_maxrss << "}\n";
  } catch (const std::exception& exception) {
    std::cerr << "benchmark failed: " << exception.what() << '\n';
    return 1;
  }
  return 0;
}
