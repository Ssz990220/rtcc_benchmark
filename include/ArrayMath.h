#pragma once

#include <algorithm>
#include <array>
#include <math.h>


template <typename T, size_t N>
std::array<T, N> operator-(const std::array<T, N>& a, const std::array<T, N>& b) {
    std::array<T, N> c;
    std::transform(a.begin(), a.end(), b.begin(), c.begin(), std::minus<T>());
    return c;
}

template <typename T, size_t N>
std::array<T, N> operator+(const std::array<T, N>& a, const std::array<T, N>& b) {
    std::array<T, N> c;
    std::transform(a.begin(), a.end(), b.begin(), c.begin(), std::plus<T>());
    return c;
}

template <typename T, size_t N>
std::array<T, N> operator*(const std::array<T, N>& a, T b) {
    std::array<T, N> c;
    std::transform(a.begin(), a.end(), c.begin(), [b](T x) { return x * b; });
    return c;
}

template <typename T, size_t N>
std::array<T, N> operator/(const std::array<T, N>& a, T b) {
    std::array<T, N> c;
    std::transform(a.begin(), a.end(), c.begin(), [b](T x) { return x / b; });
    return c;
}

template <typename T, size_t N>
std::array<T, N> operator*(T b, const std::array<T, N>& a) {
    std::array<T, N> c;
    std::transform(a.begin(), a.end(), c.begin(), [b](T x) { return x * b; });
    return c;
}

template <typename T, size_t N>
std::array<T, N> operator/(T b, const std::array<T, N>& a) {
    std::array<T, N> c;
    std::transform(a.begin(), a.end(), c.begin(), [b](T x) { return x / b; });
    return c;
}
