#ifndef UTILS_H
#define UTILS_H

#include <unistd.h>
#include <mae_global_planner/Structs.h>
#include <random>
#include <chrono>
#include <numeric>
#include <vector>
#include <algorithm>
#include <iostream>

#define logit(x) std::cout << (#x) << " = " << (x) << std::endl

inline std::vector<Point> createRandomPoints(int &&n, float &&min, float &&max)
{
    std::random_device rd;                            // obtain a random number from hardware
    std::mt19937 gen(rd());                           // seed the generator
    std::uniform_real_distribution<> distr(min, max); // define the range

    std::vector<Point> points(n);
    for (int i = 0; i < n; i++)
    {
        points[i] = Point(distr(gen), distr(gen), distr(gen));
    }
    return points;
}

template <typename T>
std::vector<T> removeDuplicates(const std::vector<T> &vec)
{
    std::vector<T> new_vec;
    for (const auto &item : vec)
    {
        if (!includes(new_vec, item))
        {
            new_vec.emplace_back(item);
        }
    }
    return new_vec;
}

inline float getRandomFloat(float &&min, float &&max)
{
    static std::random_device rd;                     // obtain a random number from hardware
    static std::mt19937 gen(rd());                    // seed the generator
    std::uniform_real_distribution<> distr(min, max); // define the range
    return distr(gen);
}

inline int getRandomInt(int &&min, int &&max)
{
    static std::random_device rd;                    // obtain a random number from hardware
    static std::mt19937 gen(rd());                   // seed the generator
    std::uniform_int_distribution<> distr(min, max); // define the range
    return distr(gen);
}

// get random index of vector
template <typename T>
inline int getRandomIndex(const std::vector<T> &vec)
{
    return getRandomInt(0, vec.size() - 1);
}

// get random element of vector
template <typename T>
inline T getRandomElement(const std::vector<T> &vec)
{
    return vec[getRandomIndex(vec)];
}

// get random element ref
template <typename T>
inline T &getRandomElementRef(std::vector<T> &vec)
{
    return vec[getRandomIndex(vec)];
}

inline void printPoint(const Point &point)
{
    std::cout << "(" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
}

template <typename T>
inline bool includes(const std::vector<T> &vec, const T &item)
{
    return std::find(vec.begin(), vec.end(), item) != vec.end();
}

inline void printPathPoints(const Path &path)
{
    for (const auto &point : path.getPoints())
    {
        printPoint(point);
    }
}
// function wrapper to count time
template <typename F, typename... Args>
void timeit(F &&f, Args &&...args)
{
    auto start = std::chrono::high_resolution_clock::now();
    f(std::forward<Args>(args)...);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Time taken by function: " << duration.count() << " milliseconds" << std::endl;
}

#endif // UTILS_H