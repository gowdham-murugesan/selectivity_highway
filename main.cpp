#include <iostream>
#include <vector>
#include <random>
#include <algorithm>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "main.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>

HWY_BEFORE_NAMESPACE();
namespace hwy
{
    namespace HWY_NAMESPACE
    {

        void SelectivityWithGatherScatter(const float *a, const float *b, float *c, const int *selectiveIndices, int numIndices, int size)
        {
            const HWY_FULL(float) df;
            const HWY_FULL(int) di;
            const int lanes = Lanes(df);
            std::cout << "Batch Size : " << lanes << std::endl;
            
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
                if (index < size)
                {
                    c[index] = a[index] + b[index];
                }
            }
        }
    }
}
HWY_AFTER_NAMESPACE();

#if HWY_ONCE
namespace hwy
{
    HWY_EXPORT(SelectivityWithGatherScatter);
    void SelectivityWithGatherScatterCall(const float *a, const float *b, float *c, const int *selectiveIndices, int numIndices, int size)
    {
        HWY_DYNAMIC_DISPATCH(SelectivityWithGatherScatter)
        (a, b, c, selectiveIndices, numIndices, size);
    }
}

const int size = 32;

void generateInputs(float *a, float *b, int *selectiveIndices, int &numIndices, int size, float selectivityPercent)
{
    std::mt19937 rng(std::random_device{}());
    std::uniform_real_distribution<float> distFloat(0.0f, 100.0f);
    std::uniform_int_distribution<int> distIndex(0, size - 1);

    for (int i = 0; i < size; i++)
    {
        a[i] = distIndex(rng);
        b[i] = distIndex(rng);
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

    std::cout << "Vector A: ";
    for (int i = 0; i < size; ++i)
    {
        std::cout << a[i] << " ";
    }
    std::cout << std::endl << std::endl;

    std::cout << "Vector B: ";
    for (int i = 0; i < size; ++i)
    {
        std::cout << b[i] << " ";
    }
    std::cout << std::endl << std::endl;

    std::cout << "Selectivity Vector: ";
    for (int i = 0; i < numIndices; ++i)
    {
        std::cout << selectiveIndices[i] << " ";
    }
    std::cout << std::endl << std::endl;
}

int main()
{
    float a[size], b[size], c[size] = {0};
    int selectiveIndices[size];
    int numIndices;

    float selectivityPercent = 50;
    generateInputs(a, b, selectiveIndices, numIndices, size, selectivityPercent);

    hwy::SelectivityWithGatherScatterCall(a, b, c, selectiveIndices, numIndices, size);

    std::cout << "Vector Addition Result: ";
    for (int i = 0; i < size; ++i)
    {
        std::cout << c[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
#endif