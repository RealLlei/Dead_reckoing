/******************************************************************************
* Copyright (c) by TL. All rights reserved.
* DO NOT ALTER OR REMOVE COPYRIGHT NOTICES OR THIS FILE HEADER.
*
* @Brief: py plot
* @Author: dingjiangang
* @Date: 2024/04/25
*****************************************************************************/

#include "planning/tasks/deciders/path_bounds_decider/util/py_plot.h"
#ifdef LMOP_PLOT_DEBUG_MSG
namespace TL::planning {

void PyPlot::AssignFolderName(const std::string& FolderName) {
    folder_name_.assign(FolderName);
}

void PyPlot::SetLineSegment(const std::vector<common::math::LineSegment2d>& line_segments) {
    freespace_line_segments_ = line_segments;
}
void PyPlot::SetLineSegmentNum(const std::vector<int>& freespace_type_num_vec) {
    freespace_type_num_vec_ = freespace_type_num_vec;
}
void PyPlot::SetTrianglePolygon(const std::vector<common::math::Polygon2d> &triangles_vec) {
    triangles_vec_ = triangles_vec;
}
void PyPlot::SetVehicleState(const common::math::Box2d& vehicle_line) {
    vehicle_line_ = vehicle_line;
}
void PyPlot::SetBoundary(int i, const std::vector<double>& x_points,
                                const std::vector<double>& y_points) {
    boundary_x_points_map_[i] = x_points;
    boundary_y_points_map_[i] = y_points;
}
void PyPlot::Plot() {
    std::stringstream counter2s;
    counter2s << counter_;
    std::string PythonFileName = folder_name_ + counter2s.str() + ending_;
    data_file_.open(PythonFileName, std::ofstream::app);
    std::fstream file(PythonFileName, std::ios::out);
    data_file_ << std::setprecision(kPrecision);
    data_file_ << "#  Author: dingjiangang" << std::endl;
    data_file_ << "import matplotlib.pyplot as plt" << std::endl << "import math" <<
                  std::endl << "import numpy as np" << std::endl;
    data_file_ << "from matplotlib.pyplot import MultipleLocator" << std::endl << std::endl;

    data_file_ << "# TODO line segment plot" << std::endl;

    data_file_ << "# TODO Plots" << std::endl;
    data_file_ << "# TODO Figure: plot freespace line segment" << std::endl;
    data_file_ << "plt.figure('freespace line segment, count: " << counter_ << "')" <<std::endl;

    data_file_ << "# we  have " << freespace_type_num_vec_.size() << " freespace type to plot" <<std::endl;
    int last_lines_num = 0;

    // to plot line segment
    for (int i = 0; i < freespace_line_segments_.size(); i++) {
        data_file_ << "line_x_" << i << "_" << counter_ << " = [";
        data_file_ << freespace_line_segments_[i].start().x() << ", ";
        data_file_ << freespace_line_segments_[i].end().x() << ", ";
        data_file_<< "]" <<std::endl;

        data_file_ << "line_y_" << i << "_" << counter_ << " = [";
        data_file_ << freespace_line_segments_[i].start().y() << ", ";
        data_file_ << freespace_line_segments_[i].end().y() << ", ";
        data_file_<< "]" <<std::endl;

        data_file_ << "plt.plot(line_x_" << i << "_" << counter_ << ", line_y_" << i << "_"
                   << counter_ << ", '.-', linewidth=0.5"
                   << ", c='r', label='freespace line segment" << counter_ << "')" << std::endl;
    }

    // to plot triangle
    for (int i = 0; i < triangles_vec_.size(); i+=5) {
        for (int j = 0; j < triangles_vec_[i].line_segments().size(); j++) {
            data_file_ << "polygon_line_x_" << i * j << "_" << counter_ << " = [";
            data_file_ << triangles_vec_[i].line_segments()[j].start().x() << ", ";
            data_file_ << triangles_vec_[i].line_segments()[j].end().x() << ", ";
            data_file_<< "]" <<std::endl;

            data_file_ << "polygon_line_y_" << i * j << "_" << counter_ << " = [";
            data_file_ << triangles_vec_[i].line_segments()[j].start().y() << ", ";
            data_file_ << triangles_vec_[i].line_segments()[j].end().y() << ", ";
            data_file_<< "]" <<std::endl;

            data_file_ << "plt.plot(polygon_line_x_" << i*j << "_" << counter_ << ", polygon_line_y_" << i*j
                       << "_" << counter_ << ", '.-', linewidth=0.5"
                       << ", c='c', label='freespace line segment" << counter_ << "')" << std::endl;
        }
    }
    std::vector<common::math::Vec2d> corners = vehicle_line_.GetAllCorners();
    data_file_ << "#  vehicle corner size is " << corners.size() << std::endl;

    if (corners.size() == 4) {
        data_file_ << "vehicle_line_x_"  << counter_ << " = [";
        for (int i = 0; i < corners.size(); i++) {
            data_file_ << corners[i].x() << ", ";
        }
        data_file_<< "]" <<std::endl;
        data_file_ << "vehicle_line_y_"  << counter_ << " = [";
        for (int i = 0; i < corners.size(); i++) {
            data_file_ << corners[i].y() << ", ";
        }
        data_file_<< "]" <<std::endl;
        data_file_ << "plt.plot(vehicle_line_x_" << counter_ << ", vehicle_line_y_" << counter_
                   << ", '.-', linewidth=0.5"
                   << ", c='k', label='freespace line segment" << counter_ << "')" << std::endl;
    }

    data_file_ << "# to plot boundary here" << std::endl;
    int index = 0;
    for (auto iter = boundary_x_points_map_.begin(); iter != boundary_x_points_map_.end(); iter++) {
        data_file_ << "# there is " << index << " boundary" <<std::endl;
        data_file_ << "boundary_line_x_"  << index << "_"<< counter_ << " = [";
        for (int i = 0; i < iter->second.size(); i++) {
            data_file_ << iter->second.at(i) << ", ";
        }
        data_file_<< "]" <<std::endl;

        data_file_ << "boundary_line_y_"  << index << "_"<< counter_ << " = [";
        for (int i = 0; i < iter->second.size(); i++) {
            data_file_ << boundary_y_points_map_[index].at(i) << ", ";
        }
        data_file_<< "]" <<std::endl;

        data_file_ << "plt.plot(boundary_line_x_" << index << "_" << counter_ << ", boundary_line_y_"
                   << index << "_" << counter_ << ", '*', linewidth=0.5"
                   << ", " << "c=" << GetColor(index) << ", label='freespace line segment"
                   << counter_ << "', markersize=2" << ")" << std::endl;
        index++;
    }
    data_file_ << "ax = plt.gca()" << std::endl;
    data_file_ << "ax.set_aspect(1)" << std::endl;
    data_file_ << "plt.show()" << std::endl;
    data_file_.close();
    counter_++;
}
}  //  namespace TL::planning
#endif
