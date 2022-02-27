#include "random_array.h"

#include <random>
#include <vector>
#include <algorithm>
#include <functional>
#include <cassert>

namespace util
{

    std::mt19937 create_random_engine()
    {
        std::random_device random_device;
        std::vector<std::uint_least32_t> v(10);
        std::generate(v.begin(), v.end(), std::ref(random_device));
        std::seed_seq seed(v.begin(), v.end());
        return std::mt19937(seed);
    }

    template <typename T>
    std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max)
    {
        assert(rand_min <= rand_max);
        assert(size <= static_cast<size_t>(rand_max - rand_min + 1));

        auto random_engine = create_random_engine();
        std::uniform_int_distribution<T> uniform_int_distribution(rand_min, rand_max);
        const auto make_size = static_cast<size_t>(size * 1.2);
        std::vector<T> v;
        v.reserve(size);
        while (v.size() != size)
        {
            while (v.size() < make_size)
            {
                v.push_back(uniform_int_distribution(random_engine));
            }

            std::sort(v.begin(), v.end());
            auto unique_end = std::unique(v.begin(), v.end());
            if (size < static_cast<size_t>(std::distance(v.begin(), unique_end)))
            {
                unique_end = std::next(v.begin(), size);
            }
            v.erase(unique_end, v.end());
        }
        std::shuffle(v.begin(), v.end(), random_engine);

        return v;
    }
    template std::vector<int> create_random_array(size_t, int, int);
    template std::vector<unsigned int> create_random_array(size_t, unsigned int, unsigned int);

}
