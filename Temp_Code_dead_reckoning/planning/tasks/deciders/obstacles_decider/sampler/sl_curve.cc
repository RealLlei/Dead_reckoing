/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2023. All rights reserved.
 * Description:  sl_curve.cc
 */

#include "planning/tasks/deciders/obstacles_decider/sampler/sl_curve.h"
#include <cmath>

#include <algorithm>
#include <cstddef>
#include <limits>
#include <sstream>

namespace TL {
namespace planning {

using State = std::array<double, 3>;
using Condition = std::pair<State, double>;

bool SlCurve::InitPiecewiseSlCurve(const std::array<double, 3>& init_lat_state,
                                   const std::vector<Condition>& l_conditions) {
  size_t piece_count = l_conditions.size();
  if (piece_count != 2) {
    return false;
  }

  l_conditions_ = l_conditions;
  l_pieces_.resize(piece_count);
  accumulated_s_.resize(piece_count + 1);

  end_l_ = 0.0;
  accumulated_s_.at(0) = 0.0;
  State init_state = init_lat_state;

  for (size_t i = 0; i < piece_count; ++i) {
    const auto end_state = l_conditions_.at(i).first;
    const auto end_s = l_conditions_.at(i).second;
    auto& l_piece = l_pieces_.at(i);
    accumulated_s_.at(i + 1) = accumulated_s_.at(i) + end_s;
    l_piece.SetParam(init_state.at(0), init_state.at(1), init_state.at(2),
                     end_state.at(0), end_state.at(1), end_state.at(2), end_s);
    end_l_ += l_piece.Evaluate(0, end_s);

    init_state = {0.0, 0.0, 0.0};
  }

  return true;
}

double SlCurve::Evaluate(const uint32_t order, const double relative_s) const {
  // piece_index is the index of the current piece ,
  // accumulated_s_.at(piece_index) is the start s of the current piece,
  // accumulated_s_.at(piece_index + 1) is the end s of the current piece
  size_t piece_index = 0;
  double accumulated_l = 0.0;
  double s = relative_s;
  const size_t piece_count = l_pieces_.size();
  if (accumulated_s_.size() != piece_count + 1) {
    return 0.0;
  }
  while (piece_index < piece_count && s > accumulated_s_.at(piece_index + 1)) {
    accumulated_l += l_pieces_.at(piece_index)
                         .Evaluate(0, accumulated_s_.at(piece_index + 1) -
                                          accumulated_s_.at(piece_index));
    ++piece_index;
  }

  if (piece_index < piece_count) {
    const double ds = s - accumulated_s_.at(piece_index);
    switch (order) {
      case 0:
        return accumulated_l + l_pieces_.at(piece_index).Evaluate(0, ds);
      case 1:
        return l_pieces_.at(piece_index).Evaluate(1, ds);
      case 2:
        return l_pieces_.at(piece_index).Evaluate(2, ds);
      case 3:
        return l_pieces_.at(piece_index).Evaluate(3, ds);
      default:
        return 0.0;
    }
  } else if (order == 0) {
    return accumulated_l;
  }

  return 0.0;
}

std::string SlCurve::DebugString() const {
  std::stringstream ss;
  ss << "piece_count:" << l_conditions_.size() << std::endl;
  for (const auto& l_piece : l_pieces_) {
    ss << "piece_coeff:" << l_piece.ToString();
  }
  return ss.str();
}

}  // namespace planning
}  // namespace TL
