/******************************************************************************
* Copyright (c)  by TL. All rights reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
*
* @Brief: py plot
* @Author: dingjiangang
* @Date: 2024/04/25
*****************************************************************************/

#ifndef TL_PY_PLOT_H
#define TL_PY_PLOT_H
#define LMOP_PLOT_DEBUG_MSG
#ifdef LMOP_PLOT_DEBUG_MSG
#pragma once
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <iomanip>
#include <map>
#include "common/math/line_segment2d.h"
#include "common/math/polygon2d.h"
#include "common/math/vec2d.h"
constexpr int kPrecision = 16;
constexpr double kRefPathErr = 0.5;
constexpr int kKappaPlotIdx = 4;
constexpr size_t kKappaPlotFrameNum = 50;
constexpr bool kPlotBoundaryInfo = true;
constexpr bool kPlotKappaInMultiFrame = false;
constexpr bool kPlotLComparison = true;
constexpr bool kPlotKappaSoftBound = true;

namespace TL::planning {

class PyPlot {
 public:
    PyPlot() : counter_(0), folder_name_("./pyFiles2/"), ending_("_PTest.py") {}

    explicit PyPlot(std::string folder_name) : counter_(0), ending_("_PTest.py") {
        folder_name_.assign(folder_name);
    }

    std::ofstream data_file_;
    int counter_;
    std::string folder_name_;
    std::string ending_;

    void AssignFolderName(const std::string &folder_name);
    void ResetCounter(int counter) {counter_ = counter;}
    void SetLineSegment(const std::vector<common::math::LineSegment2d>& line_segments);
    void SetLineSegmentNum(const std::vector<int> & line_segments_num);
    void SetTrianglePolygon(const std::vector<common::math::Polygon2d> &triangles_vec);
    void SetVehicleState(const common::math::Box2d& vehicle_line);
    void SetBoundary(int i, const std::vector<double>& x_points,
                            const std::vector<double>& y_points);
    std::string GetColor(int i) {
        if (i == 0) {
            return "'k'";
        }
        if (i == 1) {
            return "'b'";
        }
        return "'m'";
    }
    void Plot();

 private:
    std::vector<double> opt_res_l_record_ = {};
    bool plot_corridor_result_ = false;

    // linesegment
    std::vector<double> line_x_points_ = {};
    std::vector<double> line_y_points_ = {};
    std::map<int, std::vector<double>> boundary_x_points_map_;
    std::map<int, std::vector<double>> boundary_y_points_map_;
    std::vector<double> boundary_y_points_ = {};
    std::vector<common::math::LineSegment2d> freespace_line_segments_ = {};
    std::vector<int> freespace_type_num_vec_ = {};
    std::vector<common::math::Polygon2d> triangles_vec_;
    common::math::Box2d vehicle_line_;
};
}  // namespace TL::planning
#endif
#endif
