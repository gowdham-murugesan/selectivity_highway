#include <iostream>
#include <vector>
#undef HWY_TARGET_INCLUDE
#define HWY_TARGET_INCLUDE "main.cpp"
#include <hwy/foreach_target.h>
#include <hwy/highway.h>

HWY_BEFORE_NAMESPACE();
namespace hwy
{
    namespace HWY_NAMESPACE
    {
        void SelectivityWithMask(const float *a, const float *b, float *c, const float *selectiveMask, int size)
        {
            const HWY_FULL(float) d;
            auto allOne = Set(d, 1);
            std::cout << Lanes(d) << std::endl;
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

const int size = 32;

void inputGen(float *a, float *b, float *selectiveMask)
{
    for (int i = 0; i < size; i++)
    {
        a[i] = i % 1000;
        b[i] = i % 100;
        selectiveMask[i] = (i % 3) != 0;
    }
}

int main()
{
    float a[size], b[size];
    float selectiveMask[size];
    float c[size] = {0};
    inputGen(a, b, selectiveMask);
    hwy::SelectivityWithMaskCall(a, b, c, selectiveMask, size);

    std::cout << "Vector Addition Result: ";
    for (size_t i = 0; i < size; ++i)
    {
        std::cout << c[i] << " ";
    }
    std::cout << std::endl;

    return 0;
}
#endif