/*
 * Copyright (c) 2022 TL
 *
 * Author: Ling Peng
 */

#pragma once
#include <cmath>
#include <utility>

namespace TL {
namespace simdummy {

/**
 * Meyers’ Singleton
 * Asynchronous multi-thread safety after c++ 11
 *
 */
#define GET_SINGLETON(classname)         \
 public:                                 \
  static classname& getInstance() {      \
    static classname singleton_instance; \
    return singleton_instance;           \
  }

/**
 * @brief calculate the new_point value of (x2,y2) in system of (x1 ,y2 ,theta).
 * convert earth system to ego system.
 * @return the input is reasonable.
 */
bool CoordinateSystemEarth2Ego(double x1, double y1, double theta, double x2,
                               double y2,
                               std::pair<double, double>* const new_point);

/**
 * @brief calculate the new_point value of (x2,y2) in system of (x1 ,y2 ,theta).
 * convert ego system to earth system.
 * @return
 */
bool CoordinateSystemEgo2Earth(double x1, double y1, double theta, double x2,
                               double y2,
                               std::pair<double, double>* const new_point);
}  // namespace simdummy
}  // namespace TL
