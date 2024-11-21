#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "benchmark.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>
#include <benchmark/benchmark.h>

HWY_BEFORE_NAMESPACE();
namespace hwy
{
    namespace HWY_NAMESPACE
    {

        void AddWithSelectiveGatherScatterSIMD(const float *a, const float *b, float *c, const int *selectiveIndices, int numIndices, int size)
        {
            const HWY_FULL(float) df;
            const HWY_FULL(int) di;
            const int lanes = Lanes(df);

            int i = 0;
            for (; i <= numIndices - lanes; i += lanes)
            {
                auto indices = LoadU(di, selectiveIndices + i);
                auto VecA = GatherIndex(df, a, indices);
                auto VecB = GatherIndex(df, b, indices);

                auto result = VecA + VecB;

                ScatterIndex(result, df, c, indices);
            }
            for (; i < numIndices; i++)
            {
                int index = selectiveIndices[i];
                c[index] = a[index] + b[index];
            }
        }

        void AddWithSelectiveMaskSIMD(const float *a, const float *b, float *c, const float *selectiveMask, int size)
        {
            const HWY_FULL(float) d;
            auto allOne = Set(d, 1);
            int i = 0;
            for (; i < size; i += Lanes(d))
            {
                auto VecA = LoadU(d, a + i);
                auto VecB = LoadU(d, b + i);
                auto selectivityVec = LoadU(d, selectiveMask + i);
                auto selectivityMaskVec = Eq(selectivityVec, allOne);
                BlendedStore(VecA + VecB, selectivityMaskVec, d, c + i);
            }
            for (; i < size; i++)
            {
                if (selectiveMask[i])
                    c[i] = a[i] + b[i];
            }
        }
    }
}
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace hwy
{
    HWY_EXPORT(AddWithSelectiveGatherScatterSIMD);
    void ExecuteAddWithSelectiveGatherScatterSIMD(const float *a, const float *b, float *c, const int *selectiveIndices, int numIndices, int size)
    {
        HWY_DYNAMIC_DISPATCH(AddWithSelectiveGatherScatterSIMD)
        (a, b, c, selectiveIndices, numIndices, size);
    }

    HWY_EXPORT(AddWithSelectiveMaskSIMD);
    void ExecuteAddWithSelectiveMaskSIMD(const float *a, const float *b, float *c, const float *selectiveMask, int size)
    {
        HWY_DYNAMIC_DISPATCH(AddWithSelectiveMaskSIMD)
        (a, b, c, selectiveMask, size);
    }
}

void AddWithSelectiveStandard(const float *a, const float *b, float *c, const int *selectiveIndices, int numIndices, int size)
{
    for (int i = 0; i < numIndices; i++)
    {
        int index = selectiveIndices[i];
        c[index] = a[index] + b[index];
    }
}

void AddWithSelectiveMaskStandard(const float *a, const float *b, float *c, const float *selectiveMask, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (selectiveMask[i])
            c[i] = a[i] + b[i];
    }
}

const int size = 4000;

void GenerateSelectiveInputs(float *a, float *b, int *selectiveIndices, int &numIndices, float selectivityPercent)
{
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> distFloat(0.0f, 100.0f);

    for (int i = 0; i < size; i++)
    {
        a[i] = distFloat(rng);
        b[i] = distFloat(rng);
    }

    numIndices = static_cast<int>(size * (selectivityPercent / 100.0f));
    std::vector<int> indices(size);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);

    for (int i = 0; i < numIndices; i++)
    {
        selectiveIndices[i] = indices[i];
    }
    std::sort(selectiveIndices, selectiveIndices + numIndices);
}

void GenerateInputsWithSelectiveMask(float *a, float *b, float *selectiveMask, float selectivityPercent)
{
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> distFloat(0.0f, 100.0f);

    for (int i = 0; i < size; i++)
    {
        a[i] = distFloat(rng);
        b[i] = distFloat(rng);
    }

    int numSelected = static_cast<int>(size * (selectivityPercent / 100.0f));
    std::vector<int> indices(size);
    std::iota(indices.begin(), indices.end(), 0);
    std::shuffle(indices.begin(), indices.end(), rng);
    std::fill(selectiveMask, selectiveMask + size, 0.0f);

    for (int i = 0; i < numSelected; i++)
    {
        selectiveMask[indices[i]] = 1.0f;
    }
}

static void BenchmarkAddWithSelectiveGatherScatterSIMD(benchmark::State &state)
{
    float a[size], b[size], c[size] = {0};
    int selectiveIndices[size];
    int numIndices;

    float selectivityPercent = state.range(0);
    GenerateSelectiveInputs(a, b, selectiveIndices, numIndices, selectivityPercent);
    for (auto _ : state)
    {
        hwy::ExecuteAddWithSelectiveGatherScatterSIMD(a, b, c, selectiveIndices, numIndices, size);
    }
    benchmark::DoNotOptimize(c);
}

static void BenchmarkAddWithSelectiveStandard(benchmark::State &state)
{
    float a[size], b[size], c[size] = {0};
    int selectiveIndices[size];
    int numIndices;

    float selectivityPercent = state.range(0);
    GenerateSelectiveInputs(a, b, selectiveIndices, numIndices, selectivityPercent);
    for (auto _ : state)
    {
        AddWithSelectiveStandard(a, b, c, selectiveIndices, numIndices, size);
    }
    benchmark::DoNotOptimize(c);
}

static void BenchmarkAddWithSelectiveMaskSIMD(benchmark::State &state)
{
    float a[size], b[size], c[size] = {0};
    float selectiveMask[size];

    float selectivityPercent = state.range(0);
    GenerateInputsWithSelectiveMask(a, b, selectiveMask, selectivityPercent);
    for (auto _ : state)
    {
        hwy::ExecuteAddWithSelectiveMaskSIMD(a, b, c, selectiveMask, size);
    }
    benchmark::DoNotOptimize(c);
}

static void BenchmarkAddWithSelectiveMaskStandard(benchmark::State &state)
{
    float a[size], b[size], c[size] = {0};
    float selectiveMask[size];

    float selectivityPercent = state.range(0);
    GenerateInputsWithSelectiveMask(a, b, selectiveMask, selectivityPercent);
    for (auto _ : state)
    {
        AddWithSelectiveMaskStandard(a, b, c, selectiveMask, size);
    }
    benchmark::DoNotOptimize(c);
}

BENCHMARK(BenchmarkAddWithSelectiveGatherScatterSIMD)->Arg(10)->Arg(25)->Arg(50)->Arg(75)->Arg(100);
BENCHMARK(BenchmarkAddWithSelectiveStandard)->Arg(10)->Arg(25)->Arg(50)->Arg(75)->Arg(100);
BENCHMARK(BenchmarkAddWithSelectiveMaskSIMD)->Arg(10)->Arg(25)->Arg(50)->Arg(75)->Arg(100);
BENCHMARK(BenchmarkAddWithSelectiveMaskStandard)->Arg(10)->Arg(25)->Arg(50)->Arg(75)->Arg(100);
BENCHMARK_MAIN();
#endif