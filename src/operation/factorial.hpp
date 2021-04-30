#pragma once

#include <array>
#include <iostream>
#include <utility>

template<unsigned int n>
struct factorial
{
    enum
    {
        value = n * factorial<n - 1>::value
    };
};

template<>
struct factorial<0>
{
    enum { value = 1 };
};

unsigned int facto (unsigned int n) 
{
    if (n == 0 || n == 1) {
        return 1;
    }
    return facto(n - 1) * n;
}
