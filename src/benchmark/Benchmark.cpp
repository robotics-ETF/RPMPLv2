#include "Benchmark.h"
#include "State.h"
#include "RRTConnect.h"

benchmark::Benchmark::Benchmark(/* args */) {}

benchmark::Benchmark::~Benchmark() {}

void benchmark::Benchmark::addBenchmarkContext(benchmark::BenchmarkContext context)
{
    contexts.emplace_back(context);
}

void benchmark::Benchmark::runContext(BenchmarkContext context)
{
    scenario::Scenario scenario = context.first;
    std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
    if (context.second == "RRTConnect")
    {
        planning::rrt::RRTConnect planner(ss, scenario.getStart(), scenario.getGoal());
        for (size_t i = 0; i < number_of_runs; ++i)
        {
            bool res = planner.solve();
            planner.outputPlannerData(benchmark_file, false, true);
        }
    }
}

void benchmark::Benchmark::runBenchmark()
{
    for (size_t i = 0; i < contexts.size(); ++i)
    {
        runContext(contexts[i]);
    }
}