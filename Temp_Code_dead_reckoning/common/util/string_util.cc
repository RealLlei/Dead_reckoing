/******************************************************************************
 * Copyright 2017 The Apollo Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#include "common/util/string_util.h"

#include <cmath>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "common/util/span.hpp"

namespace TL {
namespace common {
namespace util {
namespace {

// static const char kBase64Array[] =
//     "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";

// std::string Base64Piece(const char in0, const char in1, const char in2) {
//   const int triplet = in0 << 16 | in1 << 8 | in2;
//   std::string out(4, '=');
//   out[0] = kBase64Array[(triplet >> 18) & 0x3f];
//   out[1] = kBase64Array[(triplet >> 12) & 0x3f];
//   if (in1) {
//     out[2] = kBase64Array[(triplet >> 6) & 0x3f];
//   }
//   if (in2) {
//     out[3] = kBase64Array[triplet & 0x3f];
//   }
//   return out;
// }

}  // namespace

// std::string EncodeBase64(std::string_view in) {
//   std::string out;
//   if (in.empty()) {
//     return out;
//   }

//   const size_t in_size = in.length();
//   out.reserve(((in_size - 1) / 3 + 1) * 4);
//   for (size_t i = 0; i + 2 < in_size; i += 3) {
//     absl::StrAppend(&out, Base64Piece(in[i], in[i + 1], in[i + 2]));
//   }
//   if (in_size % 3 == 1) {
//     absl::StrAppend(&out, Base64Piece(in[in_size - 1], 0, 0));
//   }
//   if (in_size % 3 == 2) {
//     absl::StrAppend(&out, Base64Piece(in[in_size - 2], in[in_size - 1], 0));
//   }
//   return out;
// }

// void OsqpDebug(const OSQPData* const data, const OSQPWorkspace* const work,
//                const std::string& name, const bool is_debug) {
//   if (data == nullptr || work == nullptr) {
//     AERROR << "data or work is nullptr";
//     return;
//   }

//   std::string str_temp;
//   std::stringstream ss;
//   if (work->info->status_val != 1) {
//     AERROR << " call module:" << name;
//     ADEBUG << " data_n:" << data->n << "   data_m:" << data->m
//            << "   data_p_nzmax:" << data->P->nzmax
//            << "   data_a_nzmax:" << data->A->nzmax;
//     size_t P_size = data->P->nzmax;
//     size_t P_rowcol = data->P->n;
//     size_t A_size = data->A->nzmax;
//     size_t A_col = data->A->n;
//     if (is_debug) {
//       digit2str(data->P->x, P_size, &str_temp);
//       AERROR << " std::vector<c_float> P_data=" << str_temp;

//       digit2str(data->P->i, P_size, &str_temp);
//       AERROR << " std::vector<c_int> P_indices=" << str_temp;

//       digit2str(data->P->p, P_rowcol + 1, &str_temp);
//       AERROR << " std::vector<c_int> P_indptr=" << str_temp;

//       digit2str(data->q, P_rowcol, &str_temp);
//       AERROR << " std::vector<c_float> q=" << str_temp;

//       digit2str(data->A->x, A_size, &str_temp);
//       AERROR << " std::vector<c_float> A_data=" << str_temp;

//       digit2str(data->A->i, A_size, &str_temp);
//       AERROR << " std::vector<c_int> A_indices=" << str_temp;

//       digit2str(data->A->p, A_col + 1, &str_temp);
//       AERROR << "std::vector<c_int> A_indptr=" << str_temp;

//       digit2str(data->l, data->A->m, &str_temp);
//       AERROR << "std::vector<c_float> lower_bound = " << str_temp;

//       digit2str(data->u, data->A->m, &str_temp);
//       AERROR << "std::vector<c_float> upper_bound = " << str_temp;
//     }
//     boost::span<char> status(work->info->status);
//     AERROR << "status:" << std::string(status.data());
//     AERROR << "max iterations:" << work->settings->max_iter;
//     AERROR << "time limit:" << work->settings->time_limit;
//     ADEBUG << "warm start:" << work->settings->warm_start;
//   }
//   switch (work->info->status_val) {
//     case 1:
//       // digit2str(work->solution->x, data->P->n, &str_temp);
//       // AERROR << "status_detail:OSQP_SOLVED.";
//       // AERROR << "optimal objective:"
//       //        << work->info->obj_val;
//       // AERROR << "solution_x:" << str_temp;
//       str_temp = "";
//       break;
//     case 2:
//       AERROR << "status_detail:OSQP_SOLVED_INACCURATE.";
//       str_temp = "";
//       if (is_debug) {
//         digit2str(work->solution->x, data->P->n, &str_temp);
//       }
//       AERROR << "work->solution->x = " << str_temp;
//       ss << "optimal objective:" << std::fixed << std::setprecision(3)
//          << work->info->obj_val;
//       AERROR << ss.str();
//       str_temp = "";
//       break;
//     case 3:
//       AERROR << "status_detail:OSQP_PRIMAL_INFEASIBLE_INACCURATE.";
//       break;
//     case 4:
//       AERROR << "status_detail:OSQP_DUAL_INFEASIBLE_INACCURATE.";
//       break;
//     case -2:
//       AERROR << "status_detail:OSQP_MAX_ITER_REACHED.";
//       str_temp = "";
//       ss.str("");
//       ss << "optimal objective:" << std::fixed << std::setprecision(3)
//          << work->info->obj_val;
//       AERROR << ss.str();
//       str_temp = "";
//       break;
//     case -3:
//       AERROR << "status_detail:OSQP_PRIMAL_INFEASIBLE.";
//       break;
//     case -4:
//       AERROR << "status_detail:OSQP_DUAL_INFEASIBLE.";
//       break;
//     case -5:
//       AERROR << "status_detail:OSQP_SIGINT.";
//       break;
//     case -6:
//       AERROR << "status_detail:OSQP_TIME_LIMIT_REACHED.";
//       break;
//     case -7:
//       AERROR << "status_detail:OSQP_NON_CVX.";
//       break;
//     case -10:
//       AERROR << "status_detail:OSQP_UNSOLVED.";
//       break;
//     default:
//       AERROR << "status_detail:unkown value.";
//   }
//   if (work->info->status_val != 1) {
//     AERROR << "\n";
//   }
// }

}  // namespace util
}  // namespace common
}  // namespace TL
