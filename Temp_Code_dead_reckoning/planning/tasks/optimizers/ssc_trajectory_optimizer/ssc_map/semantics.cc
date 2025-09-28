/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file semantics.cc
 **/

#include "planning/tasks/optimizers/ssc_trajectory_optimizer/ssc_map/semantics.h"

namespace TL::planning {

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND() = default;

template <typename T, int N_DIM>
GridMapND<T, N_DIM>::GridMapND(const std::array<int, N_DIM>& dims_size,
                               const std::array<double, N_DIM>& dims_resolution,
                               const std::array<std::string, N_DIM>& dims_name)
    : dims_size_(dims_size),
      dims_resolution_(dims_resolution),
      dims_name_(dims_name),
      data_(std::vector<T>(data_size_, 0)) {

  SetNDimSteps(dims_size_);
  SetDataSize(dims_size_);
  origin_.fill(0);
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::GetValueUsingCoordinate(
    const std::array<int, N_DIM>& coord, T* val) const {
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return false;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  *val = data_[idx];
  return true;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetValueUsingGlobalPosition(
    const std::array<double, N_DIM>& p_w, T* val) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  GetValueUsingCoordinate(coord, val);
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckIfEqualUsingGlobalPosition(
    const std::array<double, N_DIM>& p_w, const T& val_in, bool* res) const {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  T val;
  if (!GetValueUsingCoordinate(coord, &val)) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return true;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckIfEqualUsingCoordinate(
    const std::array<int, N_DIM>& coord, const T& val_in, bool* res) const {
  T val;
  if (!GetValueUsingCoordinate(coord, &val)) {
    *res = false;
  } else {
    *res = (val == val_in);
  }
  return true;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::SetValueUsingCoordinate(
    const std::array<int, N_DIM>& coord, const T& val) {
  if (!CheckCoordInRange(coord)) {
    // printf("[GridMapND] Out of range\n");
    return false;
  }
  int idx = GetMonoIdxUsingNDimIdx(coord);
  data_[idx] = val;
  return true;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::SetValueUsingGlobalPosition(
    const std::array<double, N_DIM>& p_w, const T& val) {
  std::array<int, N_DIM> coord = GetCoordUsingGlobalPosition(p_w);
  SetValueUsingCoordinate(coord, val);
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetCoordUsingGlobalPosition(
    const std::array<double, N_DIM>& p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord.at(i) = std::round((p_w.at(i) - origin_.at(i)) / dims_resolution_.at(i));
  }
  return coord;
}

template <typename T, int N_DIM>
std::array<double, N_DIM> GridMapND<T, N_DIM>::GetRoundedPosUsingGlobalPosition(
    const std::array<double, N_DIM>& p_w) const {
  std::array<int, N_DIM> coord = {};
  for (int i = 0; i < N_DIM; ++i) {
    coord.at(i) = std::round((p_w.at(i) - origin_.at(i)) / dims_resolution_.at(i));
  }
  std::array<double, N_DIM> round_pos = {};
  for (int i = 0; i < N_DIM; ++i) {
    round_pos.at(i) = coord.at(i) * dims_resolution_.at(i) + origin_.at(i);
  }
  return round_pos;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetGlobalPositionUsingCoordinate(
    const std::array<int, N_DIM>& coord, std::array<double, N_DIM>* p_w) const {
  for (int i = 0; i < N_DIM; ++i) {
    p_w->at(i) = coord.at(i) * dims_resolution_.at(i) + origin_.at(i);
  }
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetCoordUsingGlobalMetricOnSingleDim(
    const double& metric, const int& i, int* idx) const {
  *idx = std::round((metric - origin_.at(i)) / dims_resolution_.at(i));
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::GetGlobalMetricUsingCoordOnSingleDim(
    const int& idx, const int& i, double* metric) const {
  *metric = idx * dims_resolution_.at(i) + origin_.at(i);
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRange(
    const std::array<int, N_DIM>& coord) const {
  for (int i = 0; i < N_DIM; ++i) {
    if (coord.at(i) < 0 || coord.at(i) >= dims_size_.at(i)) {
      return false;
    }
  }
  return true;
}

template <typename T, int N_DIM>
bool GridMapND<T, N_DIM>::CheckCoordInRangeOnSingleDim(const int& idx,
                                                       const int& i) const {
  return (idx >= 0) && (idx < dims_size_.at(i));
}

template <typename T, int N_DIM>
int GridMapND<T, N_DIM>::GetMonoIdxUsingNDimIdx(
    const std::array<int, N_DIM>& idx) const {
  int mono_idx = 0;
  for (int i = 0; i < N_DIM; ++i) {
    mono_idx += dims_step_.at(i) * idx.at(i);
  }
  return mono_idx;
}

template <typename T, int N_DIM>
std::array<int, N_DIM> GridMapND<T, N_DIM>::GetNDimIdxUsingMonoIdx(
    const int& idx) const {
  std::array<int, N_DIM> idx_nd = {};
  int tmp = idx;
  for (int i = N_DIM - 1; i >= 0; --i) {
    idx_nd.at(i) = tmp / dims_step_.at(i);
    tmp = tmp % dims_step_.at(i);
  }
  return idx_nd;
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::SetNDimSteps(
    const std::array<int, N_DIM>& dims_size) {
  int step = 1;
  for (int i = 0; i < N_DIM; ++i) {
    dims_step_.at(i) = step;
    step = step * dims_size.at(i);
  }
}

template <typename T, int N_DIM>
void GridMapND<T, N_DIM>::SetDataSize(const std::array<int, N_DIM>& dims_size) {
  int total_ele_num = 1;
  for (int i = 0; i < N_DIM; ++i) {
    total_ele_num = total_ele_num * dims_size.at(i);
  }
  data_size_ = total_ele_num;
}

template class GridMapND<unsigned char, 3>;

}  // namespace TL::planning
