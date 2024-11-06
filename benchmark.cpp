#include <iostream>
#include <vector>
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
        void SelectivityWithMask(const float *a, const float *b, float *c, const float *selectiveMask, int size)
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
    HWY_EXPORT(SelectivityWithMask);
    void SelectivityWithMaskCall(const float *a, const float *b, float *c, const float *selectiveMask, int size)
    {
        HWY_DYNAMIC_DISPATCH(SelectivityWithMask)
        (a, b, c, selectiveMask, size);
    }
}

void SelectivityWithMaskNormal(const float *a, const float *b, float *c, const float *selectiveMask, int size)
{
    for (int i = 0; i < size; i++)
    {
        if (selectiveMask[i])
            c[i] = a[i] + b[i];
    }
}

const int size = 100000;

void inputGen(float *a, float *b, float *selectiveMask)
{
    for (int i = 0; i < size; i++)
    {
        a[i] = i % 1000;
        b[i] = i % 100;
        selectiveMask[i] = (i % 3) != 0;
    }
}

static void SelectivityWithMaskHighwayBenchmark(benchmark::State &state)
{
    float a[size], b[size];
    float selectiveMask[size];
    float c[size] = {0};
    inputGen(a, b, selectiveMask);
    for (auto _ : state)
    {
        hwy::SelectivityWithMaskCall(a, b, c, selectiveMask, size);
    }
    benchmark::DoNotOptimize(c);
}

static void SelectivityWithMaskNormalBenchmark(benchmark::State &state)
{
    float a[size], b[size];
    float selectiveMask[size];
    float c[size] = {0};
    inputGen(a, b, selectiveMask);
    for (auto _ : state)
    {
        SelectivityWithMaskNormal(a, b, c, selectiveMask, size);
    }
    benchmark::DoNotOptimize(c);
}

BENCHMARK(SelectivityWithMaskHighwayBenchmark);
BENCHMARK(SelectivityWithMaskNormalBenchmark);
BENCHMARK_MAIN();
#endif