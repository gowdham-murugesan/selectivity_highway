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
        void SelectivityWithMask(const float *values, const float *selectivity, float *result, int min, int max, int size)
        {
            const HWY_FULL(float) d;
            auto allLow = Set(d, min);
            auto allHigh = Set(d, max);
            auto allOne = Set(d, 1);
            int row = 0;
            for (; row < size; row += Lanes(d))
            {
                auto data = LoadU(d, values + row);
                auto ltMin = Lt(data, allLow);
                auto gtMax = Gt(data, allHigh);
                auto outOfRange = Or(ltMin, gtMax);
                if (CountTrue(d, outOfRange) != 0)
                    break;
                auto selectivityVec = LoadU(d, selectivity + row);
                auto selectivityMask = Eq(selectivityVec, allOne);
                BlendedStore(data - allLow + allOne, selectivityMask, d, result + row);
            }
            for (; row < size; row++)
            {
                auto value = values[row];
                if (value > max || value < min)
                    break;
                result[row] = selectivity[row] ? value - min + 1 : result[row];
            }
        }

        void SelectivityWithOutMask(const float *values, const float *selectivity, float *result, int min, int max, int size)
        {
            const HWY_FULL(float) d;
            auto allLow = Set(d, min);
            auto allHigh = Set(d, max);
            auto allOne = Set(d, 1);
            int row = 0;
            for (; row < size; row += Lanes(d))
            {
                auto data = LoadU(d, values + row);
                auto ltMin = Lt(data, allLow);
                auto gtMax = Gt(data, allHigh);
                auto outOfRange = Or(ltMin, gtMax);
                if (CountTrue(d, outOfRange) != 0)
                    break;
                StoreU(data - allLow + allOne, d, result + row);
            }
            for (; row < size; row++)
            {
                auto value = values[row];
                if (value > max || value < min)
                    break;
                result[row] = value - min + 1;
            }
        }
    }
}
HWY_AFTER_NAMESPACE();

#if HWY_ONCE

const int min = 0, max = 1000;
namespace hwy
{
    HWY_EXPORT(SelectivityWithMask);
    void SelectivityWithMaskCall(const float *values, const float *selectivity, float *result, int size)
    {
        HWY_DYNAMIC_DISPATCH(SelectivityWithMask)
        (values, selectivity, result, min, max, size);
    }

    HWY_EXPORT(SelectivityWithOutMask);
    void SelectivityWithOutMaskCall(const float *values, const float *selectivity, float *result, int size)
    {
        HWY_DYNAMIC_DISPATCH(SelectivityWithOutMask)
        (values, selectivity, result, min, max, size);
    }
}

void SelectivityWithMaskNormal(const float *values, const float *selectivity, float *result, int min, int max, int size)
{
    for (int row = 0; row < size; row++)
    {
        float value = values[row];
        if (value < min || value > max)
            break;
        result[row] = selectivity[row] ? value - min + 1 : result[row];
    }
}


const int size = 100000;

void inputGen(float *values, float *selectivity)
{
    for (int i = 0; i < size; i++)
    {
        values[i] = i % 1000;
        selectivity[i] = i != (i % 3);
    }
}

static void SelectivityWithMaskHighwayBenchmark(benchmark::State &state)
{
    float values[size];
    float selectivity[size];
    float result[size] = {0};
    inputGen(values, selectivity);
    for (auto _ : state)
    {
        hwy::SelectivityWithMaskCall(values, selectivity, result, size);
    }
}

static void SelectivityWithOutMaskHighwayBenchmark(benchmark::State &state)
{
    float values[size];
    float selectivity[size];
    float result[size] = {0};
    inputGen(values, selectivity);
    for (auto _ : state)
    {
        hwy::SelectivityWithOutMaskCall(values, selectivity, result, size);
    }
}

static void SelectivityWithMaskNormalBenchmark(benchmark::State &state)
{
    float values[size];
    float selectivity[size];
    float result[size] = {0};
    inputGen(values, selectivity);
    for (auto _ : state)
    {
        SelectivityWithMaskNormal(values, selectivity, result, min, max, size);
    }
}

BENCHMARK(SelectivityWithMaskHighwayBenchmark);
BENCHMARK(SelectivityWithOutMaskHighwayBenchmark);
BENCHMARK(SelectivityWithMaskNormalBenchmark);
BENCHMARK_MAIN();
#endif