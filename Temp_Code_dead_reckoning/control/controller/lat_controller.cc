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

#include "control/controller/lat_controller.h"

#include <algorithm>
#include <iomanip>
#include <utility>
#include <vector>

#include "Eigen/LU"
#include "absl/strings/str_cat.h"

#include "control/proto/lqr_matrix_k_map.pb.h"

// #include "cyber/common/environment.h"
#include "common/configs/config_gflags.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/file.h"
#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "common/math/linear_quadratic_regulator.h"
#include "common/math/math_utils.h"
#include "common/math/quaternion.h"
#include "common/time/clock.h"
#include "common/util/util.h"
#include "control/common/control_gflags.h"

namespace TL {
namespace control {

using TL::common::ErrorCode;
using TL::common::Status;
using TL::common::TrajectoryPoint;
using TL::common::VehicleStateProvider;
using Matrix = Eigen::MatrixXd;
using TL::common::Clock;

namespace {

std::string GetLogFileName() {
  time_t raw_time;
  char name_buffer[80];
  std::time(&raw_time);
  std::tm time_tm;
  localtime_r(&raw_time, &time_tm);
  strftime(name_buffer, 80, "/tmp/steer_log_simple_optimal_%F_%H%M%S.csv",
           &time_tm);
  return std::string(name_buffer);
}

void WriteHeaders(std::ofstream& file_stream) {
  file_stream << "current_lateral_error,"
              << "current_ref_heading,"
              << "current_heading,"
              << "current_heading_error,"
              << "heading_error_rate,"
              << "lateral_error_rate,"
              << "current_curvature,"
              << "steer_angle,"
              << "steer_angle_feedforward,"
              << "steer_angle_lateral_contribution,"
              << "steer_angle_lateral_rate_contribution,"
              << "steer_angle_heading_contribution,"
              << "steer_angle_heading_rate_contribution,"
              << "steer_angle_feedback,"
              << "steering_position,"
              << "v" << std::endl;
}
}  // namespace

LatController::LatController() : name_("LQR-based Lateral Controller") {
  if (FLAGS_enable_csv_debug) {
    steer_log_file_.open(GetLogFileName());
    steer_log_file_ << std::fixed;
    steer_log_file_ << std::setprecision(6);
    WriteHeaders(steer_log_file_);
  }
  AINFO << "Using " << name_;
}

LatController::~LatController() { CloseLogFile(); }

bool LatController::LoadControlConf(const ControlConf* control_conf) {
  if (!control_conf) {
    AERROR << "[LatController] control_conf == nullptr";
    return false;
  }
  lat_controller_conf_ = control_conf->lat_controller_conf();
  vehicle_param_ =
      common::VehicleConfigHelper::GetConfig().vehicle_param();

  ts_ = control_conf->lat_controller_conf().ts();
  if (ts_ <= 0.0) {
    AERROR << "[LatController] Invalid control update interval.";
    return false;
  }
  cf_ = control_conf->lat_controller_conf().cf();
  cr_ = control_conf->lat_controller_conf().cr();
  preview_window_ = control_conf->lat_controller_conf().preview_window();
  lookahead_station_low_speed_ =
      control_conf->lat_controller_conf().lookahead_station();
  lookback_station_low_speed_ =
      control_conf->lat_controller_conf().lookback_station();
  lookahead_station_high_speed_ =
      control_conf->lat_controller_conf().lookahead_station_high_speed();
  lookback_station_high_speed_ =
      control_conf->lat_controller_conf().lookback_station_high_speed();
  wheelbase_ = vehicle_param_.wheel_base();
  steer_ratio_ = vehicle_param_.steer_ratio();
  steer_single_direction_max_degree_ =
      vehicle_param_.max_steer_angle() / M_PI * 180;
  max_lat_acc_ = control_conf->lat_controller_conf().max_lateral_acceleration();
  low_speed_bound_ = control_conf_->lon_controller_conf().switch_speed();
  low_speed_window_ =
      control_conf_->lon_controller_conf().switch_speed_window();

  const double mass_fl = control_conf->lat_controller_conf().mass_fl();
  const double mass_fr = control_conf->lat_controller_conf().mass_fr();
  const double mass_rl = control_conf->lat_controller_conf().mass_rl();
  const double mass_rr = control_conf->lat_controller_conf().mass_rr();
  const double mass_front = mass_fl + mass_fr;
  const double mass_rear = mass_rl + mass_rr;
  mass_ = mass_front + mass_rear;

  lf_ = wheelbase_ * (1.0 - mass_front / mass_);
  lr_ = wheelbase_ * (1.0 - mass_rear / mass_);

  // moment of inertia
  iz_ = lf_ * lf_ * mass_front + lr_ * lr_ * mass_rear;

  lqr_eps_ = control_conf->lat_controller_conf().eps();
  lqr_max_iteration_ = control_conf->lat_controller_conf().max_iteration();

  // if (injector_->vehicle_state()->linear_velocity() < 10.0) {
  //   query_relative_time_ = 0.3;//control_conf->query_relative_time();
  // } else if (injector_->vehicle_state()->linear_velocity() >= 10.0 &&
  // injector_->vehicle_state()->linear_velocity() < 20.0){
  //   query_relative_time_ = 0.2;// * (1 -
  //   injector_->vehicle_state()->linear_velocity() / 25.0);
  // } else {
  //   query_relative_time_ = control_conf->query_relative_time();
  // }

  minimum_speed_protection_ = control_conf->minimum_speed_protection();

  return true;
}

void LatController::ProcessLogs(const SimpleLateralDebug* debug,
                                const soc::Chassis* chassis) {
  const std::string log_str = absl::StrCat(
      debug->lateral_error(), ",", debug->ref_heading(), ",", debug->heading(),
      ",", debug->heading_error(), ",", debug->heading_error_rate(), ",",
      debug->lateral_error_rate(), ",", debug->curvature(), ",",
      debug->steer_angle(), ",", debug->steer_angle_feedforward(), ",",
      debug->steer_angle_lateral_contribution(), ",",
      debug->steer_angle_lateral_rate_contribution(), ",",
      debug->steer_angle_heading_contribution(), ",",
      debug->steer_angle_heading_rate_contribution(), ",",
      debug->steer_angle_feedback(), ",", chassis->steering_percentage(), ",",
      injector_->vehicle_state()->linear_velocity());
  if (FLAGS_enable_csv_debug) {
    steer_log_file_ << log_str << std::endl;
  }
  ADEBUG << "Steer_Control_Detail: " << log_str;
}

void LatController::LogInitParameters() {
  AINFO << name_ << " begin.";
  AINFO << "[LatController parameters]"
        << " mass_: " << mass_ << ","
        << " iz_: " << iz_ << ","
        << " lf_: " << lf_ << ","
        << " lr_: " << lr_;
}

void LatController::InitializeFilters(const ControlConf* control_conf) {
  // Low pass filter
  std::vector<double> den(3, 0.0);
  std::vector<double> num(3, 0.0);
  common::LpfCoefficients(
      ts_, control_conf->lat_controller_conf().cutoff_freq(), &den, &num);
  digital_filter_.set_coefficients(den, num);
  lateral_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  heading_error_filter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
  target_heading_filiter_ = common::MeanFilter(static_cast<std::uint_fast8_t>(
      control_conf->lat_controller_conf().mean_filter_window_size()));
}

Status LatController::Init(std::shared_ptr<DependencyInjector> injector,
                           const ControlConf* control_conf) {
  control_conf_ = control_conf;
  injector_ = injector;
  if (!LoadControlConf(control_conf_)) {
    AERROR << "failed to load control conf";
    return Status(ErrorCode::CONTROL_COMPUTE_ERROR,
                  "failed to load control_conf");
  }

  InitializeFilters(control_conf_);
  auto& lat_controller_conf = control_conf_->lat_controller_conf();
  LoadLatGainScheduler(lat_controller_conf);
  LogInitParameters();
  UpdateMatrixKMap();

  enable_leadlag_ = control_conf_->lat_controller_conf()
                        .enable_reverse_leadlag_compensation();
  if (enable_leadlag_) {
    leadlag_controller_.Init(lat_controller_conf.reverse_leadlag_conf(), ts_);
  }

  enable_mrac_ =
      control_conf_->lat_controller_conf().enable_steer_mrac_control();
  if (enable_mrac_) {
    mrac_controller_.Init(lat_controller_conf.steer_mrac_conf(),
                          vehicle_param_.steering_latency_param(), ts_);
  }

  enable_look_ahead_back_control_ =
      control_conf_->lat_controller_conf().enable_look_ahead_back_control();

  return Status::OK();
}

void LatController::CloseLogFile() {
  if (FLAGS_enable_csv_debug && steer_log_file_.is_open()) {
    steer_log_file_.close();
  }
}

void LatController::LoadLatGainScheduler(
    const LatControllerConf& lat_controller_conf) {
  const auto& lat_err_gain_scheduler =
      lat_controller_conf.lat_err_gain_scheduler();
  const auto& heading_err_gain_scheduler =
      lat_controller_conf.heading_err_gain_scheduler();
  AINFO << "Lateral control gain scheduler loaded";
  Interpolation1D::DataType xy1, xy2;
  for (const auto& scheduler : lat_err_gain_scheduler.scheduler()) {
    xy1.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }
  for (const auto& scheduler : heading_err_gain_scheduler.scheduler()) {
    xy2.push_back(std::make_pair(scheduler.speed(), scheduler.ratio()));
  }

  lat_err_interpolation_.reset(new Interpolation1D);
  ACHECK(lat_err_interpolation_->Init(xy1))
      << "Fail to load lateral error gain scheduler";

  heading_err_interpolation_.reset(new Interpolation1D);
  ACHECK(heading_err_interpolation_->Init(xy2))
      << "Fail to load heading error gain scheduler";
}

void LatController::Stop() { CloseLogFile(); }

std::string LatController::Name() const { return name_; }

Status LatController::ComputeControlCommand(ControlCommand* cmd) {
  auto localization = &injector_->local_view()->localization();
  auto chassis = &(injector_->local_view()->chassis());
  auto planning_published_trajectory = &(injector_->local_view()->trajectory());
  auto vehicle_state = injector_->vehicle_state();

  auto target_tracking_trajectory = *planning_published_trajectory;

  trajectory_analyzer_ =
      std::move(TrajectoryAnalyzer(&target_tracking_trajectory));

  // Transform the coordinate of the planning trajectory from the center of the
  // rear-axis to the center of mass, if conditions matched
  trajectory_analyzer_.TrajectoryTransformToCOM(lr_);

  // Re-build the vehicle dynamic models at reverse driving (in particular,
  // replace the lateral translational motion dynamics with the corresponding
  // kinematic models)

  SimpleLateralDebug* debug = cmd->mutable_debug()->mutable_simple_lat_debug();
  debug->Clear();

  // Update state = [Lateral Error, Lateral Error Rate, Heading Error, Heading
  // Error Rate, preview lateral error1 , preview lateral error2, ...]
  UpdateState(debug);

  double v = 0.0;
  if (injector_->vehicle_state()->gear() == soc::Chassis::GEAR_REVERSE) {
    v = std::min(injector_->vehicle_state()->linear_velocity(),
                 -minimum_speed_protection_);
  } else {
    v = std::max(injector_->vehicle_state()->linear_velocity(),
                 minimum_speed_protection_);
  }
  matrix_k_ = matrix_k_map_[static_cast<int>(v / 0.1)];
  double lat_gain = lat_err_interpolation_->Interpolate(
      std::fabs(vehicle_state->linear_velocity()));
  double heading_gain = heading_err_interpolation_->Interpolate(
      std::fabs(vehicle_state->linear_velocity()));

  Eigen::MatrixXd matrix_gain;
  matrix_gain = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_gain(0, 0) = lat_gain * 1.0;
  matrix_gain(1, 1) = lat_gain * 1.0;
  matrix_gain(2, 2) = heading_gain * 0.8;
  matrix_gain(3, 3) = heading_gain * 0.8;
  const double steer_angle_feedback =
      -(matrix_k_ * matrix_gain * matrix_state_)(0, 0) * 180 / M_PI *
      steer_ratio_ / steer_single_direction_max_degree_ * 100;

  const double steer_angle_feedforward = ComputeFeedForward(debug->curvature());

  double steer_angle = 0.0;
  double steer_angle_feedback_augment = 0.0;
  // Augment the feedback control on lateral error at the desired speed domain
  if (enable_leadlag_) {
    if (FLAGS_enable_feedback_augment_on_high_speed ||
        std::fabs(vehicle_state->linear_velocity()) < low_speed_bound_) {
      steer_angle_feedback_augment =
          leadlag_controller_.Control(-matrix_state_(0, 0), ts_) * 180 / M_PI *
          steer_ratio_ / steer_single_direction_max_degree_ * 100;
      if (std::fabs(vehicle_state->linear_velocity()) >
          low_speed_bound_ - low_speed_window_) {
        // Within the low-high speed transition window, linerly interplolate the
        // augment control gain for "soft" control switch
        steer_angle_feedback_augment = common::math::lerp(
            steer_angle_feedback_augment, low_speed_bound_ - low_speed_window_,
            0.0, low_speed_bound_, std::fabs(vehicle_state->linear_velocity()));
      }
    }
  }
  steer_angle = steer_angle_feedback + steer_angle_feedforward +
                steer_angle_feedback_augment;
  // steer_angle = steer_angle_feedforward;

  // Compute the steering command limit with the given maximum lateral
  // acceleration
  const double steer_limit =
      FLAGS_set_steer_limit ? std::atan(max_lat_acc_ * wheelbase_ /
                                        (vehicle_state->linear_velocity() *
                                         vehicle_state->linear_velocity())) *
                                  steer_ratio_ * 180 / M_PI /
                                  steer_single_direction_max_degree_ * 100
                            : 100.0;

  const double steer_diff_with_max_rate =
      FLAGS_enable_maximum_steer_rate_limit
          ? vehicle_param_.max_steer_angle_rate() * ts_ * 180 / M_PI /
                steer_single_direction_max_degree_ * 100
          : 100.0;

  const double steering_position = chassis->steering_percentage();
  steer_now_ = steering_position;
  linear_velocity_ = vehicle_state->linear_velocity();

  // Re-compute the steering command if the MRAC control is enabled, with steer
  // angle limitation and steer rate limitation
  if (enable_mrac_) {
    const int mrac_model_order = control_conf_->lat_controller_conf()
                                     .steer_mrac_conf()
                                     .mrac_model_order();
    Matrix steer_state = Matrix::Zero(mrac_model_order, 1);
    steer_state(0, 0) = chassis->steering_percentage();
    if (mrac_model_order > 1) {
      steer_state(1, 0) = (steering_position - pre_steering_position_) / ts_;
    }
    if (std::fabs(vehicle_state->linear_velocity()) >
        control_conf_->minimum_speed_resolution()) {
      mrac_controller_.SetStateAdaptionRate(1.0);
      mrac_controller_.SetInputAdaptionRate(1.0);
    } else {
      mrac_controller_.SetStateAdaptionRate(0.0);
      mrac_controller_.SetInputAdaptionRate(0.0);
    }
    steer_angle = mrac_controller_.Control(
        steer_angle, steer_state, steer_limit, steer_diff_with_max_rate / ts_);
    // Set the steer mrac debug message
    MracDebug* mracdebug = debug->mutable_steer_mrac_debug();
    Matrix steer_reference = mrac_controller_.CurrentReferenceState();
    mracdebug->set_mrac_model_order(mrac_model_order);
    for (int i = 0; i < mrac_model_order; ++i) {
      mracdebug->add_mrac_reference_state(steer_reference(i, 0));
      mracdebug->add_mrac_state_error(steer_state(i, 0) -
                                      steer_reference(i, 0));
      mracdebug->mutable_mrac_adaptive_gain()->add_state_adaptive_gain(
          mrac_controller_.CurrentStateAdaptionGain()(i, 0));
    }
    mracdebug->mutable_mrac_adaptive_gain()->add_input_adaptive_gain(
        mrac_controller_.CurrentInputAdaptionGain()(0, 0));
    mracdebug->set_mrac_reference_saturation_status(
        mrac_controller_.ReferenceSaturationStatus());
    mracdebug->set_mrac_control_saturation_status(
        mrac_controller_.ControlSaturationStatus());
  }
  pre_steering_position_ = steering_position;
  debug->set_steer_mrac_enable_status(enable_mrac_);

  // Clamp the steer angle with steer limitations at current speed
  double steer_angle_limited =
      common::math::Clamp(steer_angle, -steer_limit, steer_limit);
  steer_angle = steer_angle_limited;
  debug->set_steer_angle_limited(steer_angle_limited);

  // Limit the steering command with the designed digital filter
  steer_angle = digital_filter_.Filter(steer_angle);
  // steer_angle = steer_angle + (FLAGS_steer_offset /
  // steer_single_direction_max_degree_) * 100.0;
  steer_angle = common::math::Clamp(steer_angle, -100.0, 100.0);

  // Check if the steer is locked and hence the previous steer angle should be
  // executed
  if (std::abs(vehicle_state->linear_velocity()) < FLAGS_lock_steer_speed &&
      (vehicle_state->gear() == soc::Chassis::GEAR_DRIVE ||
       vehicle_state->gear() == soc::Chassis::GEAR_REVERSE) &&
      chassis->driving_mode() == soc::Chassis::COMPLETE_AUTO_DRIVE) {
    steer_angle = pre_steer_angle_;
  }
  if (chassis->driving_mode() != soc::Chassis::COMPLETE_AUTO_DRIVE) {
    pre_steer_angle_ = steering_position;
  }
  // Set the steer commands
  cmd->set_steering_target(common::math::Clamp(
      steer_angle, pre_steer_angle_ - steer_diff_with_max_rate,
      pre_steer_angle_ + steer_diff_with_max_rate));
  cmd->set_steering_rate(FLAGS_steer_angle_rate);

  pre_steer_angle_ = cmd->steering_target();
  // compute extra information for logging and debugging
  const double steer_angle_lateral_contribution =
      -matrix_k_(0, 0) * lat_gain * matrix_state_(0, 0) * 180 / M_PI *
      steer_ratio_ / steer_single_direction_max_degree_ * 100;

  const double steer_angle_lateral_rate_contribution =
      -matrix_k_(0, 1) * lat_gain * matrix_state_(1, 0) * 180 / M_PI *
      steer_ratio_ / steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_contribution =
      -matrix_k_(0, 2) * heading_gain * matrix_state_(2, 0) * 180 / M_PI *
      steer_ratio_ / steer_single_direction_max_degree_ * 100;

  const double steer_angle_heading_rate_contribution =
      -matrix_k_(0, 3) * heading_gain * matrix_state_(3, 0) * 180 / M_PI *
      steer_ratio_ / steer_single_direction_max_degree_ * 100;

  debug->set_temp_01(-matrix_k_(0, 0));
  debug->set_temp_02(-matrix_k_(0, 1));
  debug->set_temp_03(-matrix_k_(0, 2));
  debug->set_temp_04(-matrix_k_(0, 3));
  debug->set_heading(vehicle_state->heading());
  debug->set_steer_angle(steer_angle);
  debug->set_steer_angle_feedforward(steer_angle_feedforward);
  debug->set_steer_angle_lateral_contribution(steer_angle_lateral_contribution);
  debug->set_steer_angle_lateral_rate_contribution(
      steer_angle_lateral_rate_contribution);
  debug->set_steer_angle_heading_contribution(steer_angle_heading_contribution);
  debug->set_steer_angle_heading_rate_contribution(
      steer_angle_heading_rate_contribution);
  debug->set_steer_angle_feedback(steer_angle_feedback);
  debug->set_steer_angle_feedback_augment(steer_angle_feedback_augment);
  debug->set_steering_position(steering_position);
  debug->set_ref_speed(vehicle_state->linear_velocity());
  debug->set_steer_offset(vehicle_param_.steer_offset());

  ProcessLogs(debug, chassis);
  return Status::OK();
}

Status LatController::Reset() {
  matrix_state_.setZero();
  if (enable_mrac_) {
    mrac_controller_.Reset();
  }
  return Status::OK();
}

void LatController::UpdateState(SimpleLateralDebug* debug) {
  auto vehicle_state = injector_->vehicle_state();
  // Transform the coordinate of the vehicle states from the center of the
  // rear-axis to the center of mass, if conditions matched
  double passed_time =
      common::Clock::NowInSeconds() - vehicle_state->timestamp();
  auto vehicle_state_proto = vehicle_state->vehicle_state();
  VehicleStateProvider::EstimateFuturePosition(passed_time,
                                               &vehicle_state_proto);
  auto com = common::util::ComputeCOMPosition(
      lr_,
      common::math::Vec2d(vehicle_state_proto.x(), vehicle_state_proto.y()),
      vehicle_state_proto.pose().quaternion());
  ComputeLateralErrors(
      com.x(), com.y(), vehicle_state_proto.heading(),
      vehicle_state->linear_velocity(), vehicle_state->angular_velocity(),
      vehicle_state->linear_acceleration(), trajectory_analyzer_, debug);

  matrix_state_(0, 0) = debug->lateral_error();
  matrix_state_(2, 0) = debug->heading_error();
  matrix_state_(1, 0) = debug->lateral_error_rate();
  matrix_state_(3, 0) = debug->heading_error_rate();

  // Next elements are depending on preview window size;
  for (int i = 0; i < preview_window_; ++i) {
    const double preview_time = ts_ * (i + 1);
    const auto preview_point =
        trajectory_analyzer_.QueryNearestPointByRelativeTime(preview_time);

    const auto matched_point = trajectory_analyzer_.QueryNearestPointByPosition(
        preview_point.path_point().x(), preview_point.path_point().y());

    const double dx =
        preview_point.path_point().x() - matched_point.path_point().x();
    const double dy =
        preview_point.path_point().y() - matched_point.path_point().y();

    const double cos_matched_theta =
        std::cos(matched_point.path_point().theta());
    const double sin_matched_theta =
        std::sin(matched_point.path_point().theta());
    const double preview_d_error =
        cos_matched_theta * dy - sin_matched_theta * dx;

    matrix_state_(basic_state_size_ + i, 0) = preview_d_error;
  }
}

void LatController::UpdateMatrix(const double v) {
  if (v < 0) {
    /*
    A matrix (Gear Reverse)
    [0.0, 0.0, 1.0 * v 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = -control_conf_->lat_controller_conf().cf();
    cr_ = -control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 0.0;
    matrix_a_coeff_(0, 2) = 1.0;
  } else {
    /*
    A matrix (Gear Drive)
    [0.0, 1.0, 0.0, 0.0;
     0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
     (l_r * c_r - l_f * c_f) / m / v;
     0.0, 0.0, 0.0, 1.0;
     0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
     (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
    */
    cf_ = control_conf_->lat_controller_conf().cf();
    cr_ = control_conf_->lat_controller_conf().cr();
    matrix_a_(0, 1) = 1.0;
    matrix_a_coeff_(0, 2) = 0.0;
  }
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;
  if (FLAGS_reverse_heading_control && v < 0) {
    matrix_bd_ = -matrix_b_ * ts_;
    // Update Matrix_b for reverse mode
    ADEBUG << "Matrix_b changed due to gear direction";
  }
  // At reverse driving, replace the lateral translational motion dynamics with
  // the corresponding kinematic models
  if (v < 0) {
    matrix_a_(0, 2) = matrix_a_coeff_(0, 2) * v;
  } else {
    matrix_a_(0, 2) = 0.0;
  }
  matrix_a_(1, 1) = matrix_a_coeff_(1, 1) / v;
  matrix_a_(1, 3) = matrix_a_coeff_(1, 3) / v;
  matrix_a_(3, 1) = matrix_a_coeff_(3, 1) / v;
  matrix_a_(3, 3) = matrix_a_coeff_(3, 3) / v;
  Matrix matrix_i = Matrix::Identity(matrix_a_.cols(), matrix_a_.cols());
  matrix_ad_ = (matrix_i - ts_ * 0.5 * matrix_a_).inverse() *
               (matrix_i + ts_ * 0.5 * matrix_a_);
}

void LatController::UpdateMatrixCompound() {
  // Initialize preview matrix
  matrix_adc_.block(0, 0, basic_state_size_, basic_state_size_) = matrix_ad_;
  matrix_bdc_.block(0, 0, basic_state_size_, 1) = matrix_bd_;
  if (preview_window_ > 0) {
    matrix_bdc_(matrix_bdc_.rows() - 1, 0) = 1;
    // Update A matrix;
    for (int i = 0; i < preview_window_ - 1; ++i) {
      matrix_adc_(basic_state_size_ + i, basic_state_size_ + 1 + i) = 1;
    }
  }
}

double LatController::ComputeFeedForward(double ref_curvature) const {
  const double kv =
      lr_ * mass_ / 2 / cf_ / wheelbase_ - lf_ * mass_ / 2 / cr_ / wheelbase_;

  // Calculate the feedforward term of the lateral controller; then change it
  // from rad to %
  const double v = injector_->vehicle_state()->linear_velocity();
  double steer_angle_feedforwardterm;
  if (injector_->vehicle_state()->gear() == soc::Chassis::GEAR_REVERSE) {
    steer_angle_feedforwardterm = wheelbase_ * ref_curvature * 180 / M_PI *
                                  steer_ratio_ /
                                  steer_single_direction_max_degree_ * 100;
  } else {
    steer_angle_feedforwardterm =
        (wheelbase_ * ref_curvature + kv * v * v * ref_curvature -
         matrix_k_(0, 2) *
             (lr_ * ref_curvature -
              lf_ * mass_ * v * v * ref_curvature / 2 / cr_ / wheelbase_)) *
        180 / M_PI * steer_ratio_ / steer_single_direction_max_degree_ * 100;
  }

  return steer_angle_feedforwardterm;
}

void LatController::ComputeLateralErrors(
    const double x, const double y, const double theta, const double linear_v,
    const double angular_v, const double linear_a,
    const TrajectoryAnalyzer& trajectory_analyzer, SimpleLateralDebug* debug) {
  TrajectoryPoint target_point;
  TrajectoryPoint current_point;
  TrajectoryPoint preview_point;

  current_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
  target_point = trajectory_analyzer.QueryNearestPointByPosition(x, y);
  double preview_time = common::math::InterpolationOne(
      injector_->vehicle_state()->linear_velocity(),
      lat_controller_conf_.speed_segment(),
      lat_controller_conf_.preview_time());
  AINFO << "preview_time: " << preview_time;
  preview_point = trajectory_analyzer.QueryNearestPointByAbsoluteTime(
      Clock::NowInSeconds() + preview_time);
  const double dx = x - target_point.path_point().x();
  const double dy = y - target_point.path_point().y();

  debug->mutable_current_target_point()->mutable_path_point()->set_x(
      target_point.path_point().x());
  debug->mutable_current_target_point()->mutable_path_point()->set_y(
      target_point.path_point().y());

  ADEBUG << "x point: " << x << " y point: " << y;
  ADEBUG << "match point information : " << target_point.ShortDebugString();

  const double cos_target_heading = std::cos(target_point.path_point().theta());
  const double sin_target_heading = std::sin(target_point.path_point().theta());

  double lateral_error = cos_target_heading * dy - sin_target_heading * dx;
  if (FLAGS_enable_navigation_mode_error_filter) {
    lateral_error = lateral_error_filter_.Update(lateral_error);
  }

  debug->set_lateral_error(lateral_error);
  double target_heading =
      target_heading_filiter_.Update(target_point.path_point().theta());
  debug->set_ref_heading(target_point.path_point().theta());
  double heading_error =
      common::math::NormalizeAngle(theta - debug->ref_heading());
  if (FLAGS_enable_navigation_mode_error_filter) {
    heading_error = heading_error_filter_.Update(heading_error);
  }
  debug->set_heading_error(heading_error);

  auto lateral_error_dot = linear_v * std::sin(heading_error);
  auto lateral_error_dot_dot = linear_a * std::sin(heading_error);
  if (FLAGS_reverse_heading_control) {
    if (injector_->vehicle_state()->gear() == soc::Chassis::GEAR_REVERSE) {
      lateral_error_dot = -lateral_error_dot;
      lateral_error_dot_dot = -lateral_error_dot_dot;
    }
  }
  debug->set_lateral_error_rate(lateral_error_dot);
  debug->set_lateral_acceleration(lateral_error_dot_dot);
  debug->set_lateral_jerk(
      (debug->lateral_acceleration() - previous_lateral_acceleration_) / ts_);
  previous_lateral_acceleration_ = debug->lateral_acceleration();

  if (injector_->vehicle_state()->gear() == soc::Chassis::GEAR_REVERSE) {
    debug->set_heading_rate(-angular_v);
  } else {
    debug->set_heading_rate(angular_v);
  }
  debug->set_ref_heading_rate(target_point.path_point().kappa() *
                              target_point.v());
  debug->set_heading_error_rate(debug->heading_rate() -
                                debug->ref_heading_rate());

  debug->set_heading_acceleration(
      (debug->heading_rate() - previous_heading_rate_) / ts_);
  debug->set_ref_heading_acceleration(
      (debug->ref_heading_rate() - previous_ref_heading_rate_) / ts_);
  debug->set_heading_error_acceleration(debug->heading_acceleration() -
                                        debug->ref_heading_acceleration());
  previous_heading_rate_ = debug->heading_rate();
  previous_ref_heading_rate_ = debug->ref_heading_rate();

  debug->set_heading_jerk(
      (debug->heading_acceleration() - previous_heading_acceleration_) / ts_);
  debug->set_ref_heading_jerk(
      (debug->ref_heading_acceleration() - previous_ref_heading_acceleration_) /
      ts_);
  debug->set_heading_error_jerk(debug->heading_jerk() -
                                debug->ref_heading_jerk());
  previous_heading_acceleration_ = debug->heading_acceleration();
  previous_ref_heading_acceleration_ = debug->ref_heading_acceleration();

  debug->set_curvature(preview_point.path_point().kappa());
}

void LatController::UpdateMatrixKMap() {
  // Matrix init operations.
  const int matrix_size = basic_state_size_ + preview_window_;
  matrix_a_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_ad_ = Matrix::Zero(basic_state_size_, basic_state_size_);
  matrix_adc_ = Matrix::Zero(matrix_size, matrix_size);
  /*
  A matrix (Gear Drive)
  [0.0, 1.0, 0.0, 0.0;
   0.0, (-(c_f + c_r) / m) / v, (c_f + c_r) / m,
   (l_r * c_r - l_f * c_f) / m / v;
   0.0, 0.0, 0.0, 1.0;
   0.0, ((lr * cr - lf * cf) / i_z) / v, (l_f * c_f - l_r * c_r) / i_z,
   (-1.0 * (l_f^2 * c_f + l_r^2 * c_r) / i_z) / v;]
  */
  matrix_a_(0, 1) = 1.0;
  matrix_a_(1, 2) = (cf_ + cr_) / mass_;
  matrix_a_(2, 3) = 1.0;
  matrix_a_(3, 2) = (lf_ * cf_ - lr_ * cr_) / iz_;

  matrix_a_coeff_ = Matrix::Zero(matrix_size, matrix_size);
  matrix_a_coeff_(1, 1) = -(cf_ + cr_) / mass_;
  matrix_a_coeff_(1, 3) = (lr_ * cr_ - lf_ * cf_) / mass_;
  matrix_a_coeff_(3, 1) = (lr_ * cr_ - lf_ * cf_) / iz_;
  matrix_a_coeff_(3, 3) = -1.0 * (lf_ * lf_ * cf_ + lr_ * lr_ * cr_) / iz_;

  /*
  b = [0.0, c_f / m, 0.0, l_f * c_f / i_z]^T
  */
  matrix_b_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bd_ = Matrix::Zero(basic_state_size_, 1);
  matrix_bdc_ = Matrix::Zero(matrix_size, 1);
  matrix_b_(1, 0) = cf_ / mass_;
  matrix_b_(3, 0) = lf_ * cf_ / iz_;
  matrix_bd_ = matrix_b_ * ts_;

  matrix_state_ = Matrix::Zero(matrix_size, 1);
  matrix_k_ = Matrix::Zero(1, matrix_size);
  matrix_r_ = Matrix::Identity(1, 1);
  matrix_q_ = Matrix::Zero(matrix_size, matrix_size);

  int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
  int reverse_q_param_size =
      control_conf_->lat_controller_conf().reverse_matrix_q_size();
  if (matrix_size != q_param_size || matrix_size != reverse_q_param_size) {
    const auto error_msg = absl::StrCat(
        "lateral controller error: matrix_q size: ", q_param_size,
        "lateral controller error: reverse_matrix_q size: ",
        reverse_q_param_size,
        " in parameter file not equal to matrix_size: ", matrix_size);
    AERROR << error_msg;
    return;
  }

  for (int i = 0; i < q_param_size; ++i) {
    matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
  }
  matrix_q_updated_ = matrix_q_;
  LQRMatrixKMAP matrix_k_map_pb;
  if (common::GetProtoFromFile(FLAGS_matrix_k_map_file, &matrix_k_map_pb)) {
    AINFO << "use matrix K map file existed";
    for (int i = 0; i < matrix_k_map_pb.matrix_k_size() - 1; i++) {
      matrix_k_(0, 0) = matrix_k_map_pb.matrix_k(i).element(0);
      matrix_k_(0, 1) = matrix_k_map_pb.matrix_k(i).element(1);
      matrix_k_(0, 2) = matrix_k_map_pb.matrix_k(i).element(2);
      matrix_k_(0, 3) = matrix_k_map_pb.matrix_k(i).element(3);
      matrix_k_map_[static_cast<int>(
          round(matrix_k_map_pb.matrix_k(i).speed() / 0.1))] = matrix_k_;
      // AINFO << "LQR matrix K: v " <<
      // static_cast<int>(round(matrix_k_map_pb.matrix_k(i).speed() /
      //                                0.1)) << " "
      //       << matrix_k_(0, 0) << " " << matrix_k_(0, 1) << " "
      //       << matrix_k_(0, 2) << " " << matrix_k_(0, 3) << matrix_k_;
    }
    return;
  } else {
    AWARN << "Unable to load matrix k map_file file: " +
                 FLAGS_matrix_k_map_file + ", generate new one.";
  }

  for (int i = -50; i <= 400; i++) {
    double v = 0.1 * i;
    if (fabs(v) < 0.05) {
      v = 0.1;
    }
    UpdateMatrix(v);

    // Compound discrete matrix with road preview model
    UpdateMatrixCompound();

    // Adjust matrix_q_updated when in reverse gear
    int q_param_size = control_conf_->lat_controller_conf().matrix_q_size();
    int reverse_q_param_size =
        control_conf_->lat_controller_conf().reverse_matrix_q_size();
    if (v < 0) {
      for (int i = 0; i < reverse_q_param_size; ++i) {
        matrix_q_(i, i) =
            control_conf_->lat_controller_conf().reverse_matrix_q(i);
      }
    } else {
      for (int i = 0; i < q_param_size; ++i) {
        matrix_q_(i, i) = control_conf_->lat_controller_conf().matrix_q(i);
      }
    }

    // Add gain scheduler for higher speed steering
    if (FLAGS_enable_gain_scheduler) {
      matrix_q_updated_(0, 0) =
          matrix_q_(0, 0) * lat_err_interpolation_->Interpolate(std::fabs(v));
      matrix_q_updated_(2, 2) =
          matrix_q_(2, 2) *
          heading_err_interpolation_->Interpolate(std::fabs(v));
      common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_updated_,
                                    matrix_r_, lqr_eps_, lqr_max_iteration_,
                                    &matrix_k_);
      AINFO << "update v " << v << " lat "
            << lat_err_interpolation_->Interpolate(std::fabs(v)) << " heading "
            << heading_err_interpolation_->Interpolate(std::fabs(v));
    } else {
      common::math::SolveLQRProblem(matrix_adc_, matrix_bdc_, matrix_q_,
                                    matrix_r_, lqr_eps_, lqr_max_iteration_,
                                    &matrix_k_);
    }
    matrix_k_map_[i] = matrix_k_;
    auto matrix_k = matrix_k_map_pb.add_matrix_k();
    matrix_k->set_speed(0.1 * i);
    matrix_k->add_element(matrix_k_(0, 0));
    matrix_k->add_element(matrix_k_(0, 1));
    matrix_k->add_element(matrix_k_(0, 2));
    matrix_k->add_element(matrix_k_(0, 3));

    AINFO << "LQR matrix K: v " << 0.1 * i << " " << matrix_k_(0, 0) << " "
          << matrix_k_(0, 1) << " " << matrix_k_(0, 2) << " "
          << matrix_k_(0, 3);
  }
  const std::string txt_file = FLAGS_matrix_k_map_file;
  ACHECK(TL::common::SetProtoToASCIIFile(matrix_k_map_pb, txt_file));
}

void LatController::PIDCalculateFunc() {
  auto chassis = &(injector_->local_view()->chassis());
  const SteerTorqueControllerConf& steer_torque_controller_conf =
      lat_controller_conf_.steer_torque_controller_conf();
  double pid_ts = steer_torque_controller_conf.pid_ts();
  double steer_error_limit = steer_torque_controller_conf.steer_error_limit();
  double torque_error_limit = steer_torque_controller_conf.torque_error_limit();
  double steer_angle_error_limited = 0.0;
  steer_angle_error_limited = common::math::Clamp(
      steer_target_ - steer_now_, -steer_error_limit, steer_error_limit);
  double feedforward_k = 0.0;
  if (linear_velocity_ >= steer_torque_controller_conf.switch_speed()) {
    steer_pid_controller_.SetPID(
        steer_torque_controller_conf.steer_highspeed_pid_conf());
    feedforward_k =
        steer_torque_controller_conf.steer_highspeed_pid_conf().kff();
  } else {
    steer_pid_controller_.SetPID(
        steer_torque_controller_conf.steer_lowspeed_pid_conf());
    feedforward_k =
        steer_torque_controller_conf.steer_lowspeed_pid_conf().kff();
  }
  double steer_torque_feedforward = feedforward_k * steer_target_;
  // y = ax^2+bx+c 期望： 原点o到点B之间上升速度快，B到A之间的上升缓慢
  // 取点A(90, a_point_y), B(10, b_point_y)
  double a_point_y = 0.6;
  double b_point_y = 0.2;
  double a = (a_point_y - 9 * b_point_y) / 7200.0;
  double b = (b_point_y - 100 * a) / 10.0;
  double x = std::fabs(steer_target_);
  double y = a * x * x + b * x;
  steer_torque_feedforward = steer_target_ > 0.0 ? y : -y;

  double steer_feedback =
      steer_pid_controller_.Control(steer_angle_error_limited, pid_ts);
  double steer_torque = steer_feedback + steer_torque_feedforward;
  // 右转向
  if (steer_target_ < 0.0) {
    steer_torque = 0.80 * steer_torque;
  }
  static double previous_steer_cmd = 0.0;
  steer_torque = common::math::Clamp(steer_torque, previous_steer_cmd - 0.01,
                                     previous_steer_cmd + 0.01);
  steer_pid_cmd_ = common::math::Clamp(steer_torque, -torque_error_limit,
                                       torque_error_limit);
  if (std::fabs(steer_now_) > 100.0) {
    steer_pid_cmd_ = 0.0;
  }
  previous_steer_cmd = steer_pid_cmd_;

  if (chassis->driving_mode() == soc::Chassis::COMPLETE_AUTO_DRIVE) {
    AERROR << "PID: v:" << linear_velocity_
           << " steer_target_: " << steer_target_
           << " steer_now_:" << steer_now_
           << " steer_angle_error_limited:" << steer_angle_error_limited
           << " feedforward " << steer_torque_feedforward
           << " steer_feedback:" << steer_feedback
           << " steer_cmd:" << steer_pid_cmd_;
  }
}

void LatController::CalibrationFunc() {
  auto chassis = &(injector_->local_view()->chassis());
  const SteerTorqueControllerConf& steer_torque_controller_conf =
      lat_controller_conf_.steer_torque_controller_conf();
  double steer_lowerbound =
      std::max(vehicle_param_.steer_deadzone(),
               steer_torque_controller_conf.steer_minimum_action());
  double calibration_value = 0.0;
  double angle_lookup = 0.0;
  angle_lookup = steer_target_;
  calibration_value = control_interpolation_->Interpolate(
      std::make_pair(linear_velocity_, angle_lookup));

  if (calibration_value >= 0) {
    steer_calibration_cmd_ = std::max(calibration_value, steer_lowerbound);
  } else {
    steer_calibration_cmd_ = steer_lowerbound;
  }

  if (chassis->driving_mode() == soc::Chassis::COMPLETE_AUTO_DRIVE) {
    //   AERROR << "calibration: v:" << linear_velocity_ << " steer_target_: "
    //   << steer_target_
    //        << " steer_now_:" << steer_now_
    //        << " calibration_value:" << calibration_value
    //        << " steer_calibration_cmd:" << steer_calibration_cmd_;
  }
}

}  // namespace control
}  // namespace TL
