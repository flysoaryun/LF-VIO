#ifndef VINS_UTIL_RANDOM_ARRAY_H
#define VINS_UTIL_RANDOM_ARRAY_H

#include <vector>
#include <random>

namespace util
{

    std::mt19937 create_random_engine();

    template <typename T>
    std::vector<T> create_random_array(const size_t size, const T rand_min, const T rand_max);

}

#endif
