
/*
 * Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.
 * Description:  control teleop
 * Author: ROC
 */

#include <termios.h>

#include <algorithm>
#include <cstdio>
#include <iostream>
#include <memory>
#include <thread>

#include "canbus/vehicle/vehicle_controller.h"
#include "common/adapters/adapter_gflags.h"
#include "common/configs/vehicle_config_helper.h"
#include "common/file/log.h"
#include "common/math/linear_interpolation.h"
#include "common/math/math_utils.h"
#include "common/util/macros.h"
#include "common/util/message_util.h"
#include "control/common/control_gflags.h"
#include "control/common/interpolation_1d.h"
#include "control/common/interpolation_2d.h"
#include "control/common/pid_controller.h"

#include "proto/soc/chassis.pb.h"
#include "control/proto/control_cmd.pb.h"
#include "control/proto/control_conf.pb.h"
namespace Teleop {

using TL::soc::Chassis;
using TL::common::VehicleSignal;
using TL::control::ControlCommand;
using TL::control::PadMessage;

const uint32_t KEYCODE_O = 0x4F;  // '0'

const uint32_t KEYCODE_UP1 = 0x57;  // 'w'
const uint32_t KEYCODE_UP2 = 0x77;  // 'w'
const uint32_t KEYCODE_DN1 = 0x53;  // 'S'
const uint32_t KEYCODE_DN2 = 0x73;  // 's'
const uint32_t KEYCODE_LF1 = 0x41;  // 'A'
const uint32_t KEYCODE_LF2 = 0x61;  // 'a'
const uint32_t KEYCODE_RT1 = 0x44;  // 'D'
const uint32_t KEYCODE_RT2 = 0x64;  // 'd'

const uint32_t KEYCODE_PKBK = 0x50;  // hand brake or parking brake

// set throttle, gear, and brake
const uint32_t KEYCODE_SETT1 = 0x54;  // 'T'
const uint32_t KEYCODE_SETT2 = 0x74;  // 't'
const uint32_t KEYCODE_SETG1 = 0x47;  // 'G'
const uint32_t KEYCODE_SETG2 = 0x67;  // 'g'
const uint32_t KEYCODE_SETB1 = 0x42;  // 'B'
const uint32_t KEYCODE_SETB2 = 0x62;  // 'b'
const uint32_t KEYCODE_ZERO = 0x30;   // '0'

const uint32_t KEYCODE_SETQ1 = 0x51;  // 'Q'
const uint32_t KEYCODE_SETQ2 = 0x71;  // 'q'

// change action
const uint32_t KEYCODE_MODE1 = 0x4D;  // 'M'
const uint32_t KEYCODE_MODE2 = 0x6D;  // 'm'

// emergency stop
const uint32_t KEYCODE_ESTOP = 0x45;  // 'E'

// help
const uint32_t KEYCODE_HELP = 0x68;   // 'h'
const uint32_t KEYCODE_HELP2 = 0x48;  // 'H'

// set ctrl mode
const uint32_t KEYCODE_Z1 = 0X5A;  // Z
const uint32_t KEYCODE_Z2 = 0X7A;  // z
class Teleop {
 public:
  Teleop() { ResetControlCommand(); }
  static void PrintKeycode() {
    system("clear");
    printf("=====================    KEYBOARD MAP   ===================\n");
    printf("HELP:               [%c]     |\n", KEYCODE_HELP);
    printf("Set Action      :   [%c]+Num\n", KEYCODE_MODE1);
    printf("                     0 RESET ACTION\n");
    printf("                     1 START ACTION\n");
    printf("\n-----------------------------------------------------------\n");
    printf("Set Gear:           [%c]+Num\n", KEYCODE_SETG1);
    printf("                     0 GEAR_NEUTRAL\n");
    printf("                     1 GEAR_DRIVE\n");
    printf("                     2 GEAR_REVERSE\n");
    printf("                     3 GEAR_PARKING\n");
    printf("                     4 GEAR_LOW\n");
    printf("                     5 GEAR_INVALID\n");
    printf("                     6 GEAR_NONE\n");
    printf("\n-----------------------------------------------------------\n");
    printf("Throttle/Speed up:  [%c]     |  Set Throttle:       [%c]+Num\n",
           KEYCODE_UP1, KEYCODE_SETT1);
    printf("ControlMode:  [%c]           |  Set ControlMode:    [%c]+Num\n",
           KEYCODE_Z1, KEYCODE_Z2);
    printf("Brake/Speed down:   [%c]     |  Set Brake:          [%c]+Num\n",
           KEYCODE_DN1, KEYCODE_SETB1);
    printf("Steer LEFT:         [%c]     |  Steer RIGHT:        [%c]\n",
           KEYCODE_LF1, KEYCODE_RT1);
    printf("Parking Brake:     [%c]     |  Emergency Stop      [%c]\n",
           KEYCODE_PKBK, KEYCODE_ESTOP);
    printf("\n-----------------------------------------------------------\n");
    printf("Exit: Ctrl + C, then press enter to normal terminal\n");
    printf("===========================================================\n");
  }

  void KeyboardLoopThreadFunc() {
    char c = 0;
    int32_t level = 0;
    double brake = 0;
    double throttle = 0;
    double acc = 0;
    double dec = 0;
    double steering = 0;
    double steer_torque = 0;
    struct termios cooked_;
    struct termios raw_;
    int32_t kfd_ = 0;
    bool parking_brake = false;
    Chassis::GearPosition gear = Chassis::GEAR_INVALID;
    PadMessage pad_msg;
    ControlCommand& control_command_ = control_command();
    vehicle_params_ = TL::common::VehicleConfigHelper::GetConfig()
                          .vehicle_param();

    // get the console in raw mode
    tcgetattr(kfd_, &cooked_);
    std::memcpy(&raw_, &cooked_, sizeof(struct termios));
    raw_.c_lflag &= ~(ICANON | ECHO);
    // Setting a new line, then end of file
    raw_.c_cc[VEOL] = 1;
    raw_.c_cc[VEOF] = 2;
    tcsetattr(kfd_, TCSANOW, &raw_);
    puts("Teleop:\nReading from keyboard now.");
    puts("---------------------------");
    puts("Use arrow keys to drive the car.");
    while (IsRunning()) {
      // get the next event from the keyboard
      if (read(kfd_, &c, 1) < 0) {
        perror("read():");
        exit(-1);
      }
      // AINFO << "control command : "
      //       << control_command_.ShortDebugString().c_str();
      switch (c) {
        case KEYCODE_UP1:  // accelerate
        case KEYCODE_UP2:
          if (!FLAGS_use_acceleration) {
            brake = control_command_.brake();
            throttle = control_command_.throttle();
          }
          if (brake > 1e-6) {
            brake = GetCommand(brake, -FLAGS_brake_inc_delta);
            if (!FLAGS_use_acceleration) {
              control_command_.set_brake(brake);
            } else {
              dec = brake / 100 * vehicle_params_.max_deceleration();
              if (control_command_.gear_location() == Chassis::GEAR_REVERSE) {
                dec = -dec;
              }
              control_command_.set_acceleration(dec);
            }
          } else {
            throttle = GetCommand(throttle, FLAGS_throttle_inc_delta);
            if (!FLAGS_use_acceleration) {
              control_command_.set_throttle(throttle);
            } else {
              acc = throttle / 100 * vehicle_params_.max_acceleration();
              if (control_command_.gear_location() == Chassis::GEAR_REVERSE) {
                acc = -acc;
              }
              control_command_.set_acceleration(acc);
            }
          }
          if (!FLAGS_use_acceleration) {
            AINFO << "Throttle = " << control_command_.throttle()
                  << ", Brake = " << control_command_.brake();
          } else {
            AINFO << "Acceleration = " << control_command_.acceleration();
          }
          break;
        case KEYCODE_DN1:  // decelerate
        case KEYCODE_DN2:
          if (!FLAGS_use_acceleration) {
            brake = control_command_.brake();
            throttle = control_command_.throttle();
          }
          if (throttle > 1e-6) {
            throttle = GetCommand(throttle, -FLAGS_throttle_inc_delta);
            if (!FLAGS_use_acceleration) {
              control_command_.set_throttle(throttle);
            } else {
              acc = throttle / 100 * vehicle_params_.max_acceleration();
              if (control_command_.gear_location() == Chassis::GEAR_REVERSE) {
                acc = -acc;
              }
              control_command_.set_acceleration(acc);
            }
          } else {
            brake = GetCommand(brake, FLAGS_brake_inc_delta);
            if (!FLAGS_use_acceleration) {
              control_command_.set_brake(brake);
            } else {
              dec = brake / 100 * vehicle_params_.max_deceleration();
              if (control_command_.gear_location() == Chassis::GEAR_REVERSE) {
                dec = -dec;
              }
              control_command_.set_acceleration(dec);
            }
          }
          if (!FLAGS_use_acceleration) {
            AINFO << "Throttle = " << control_command_.throttle()
                  << ", Brake = " << control_command_.brake();
          } else {
            AINFO << "Acceleration = " << control_command_.acceleration();
          }
          break;
        case KEYCODE_LF1:  // left
        case KEYCODE_LF2:
          steering = control_command_.steering_target();
          steering = GetCommand(steering, FLAGS_steer_inc_delta);
          if (!FLAGS_enable_teleop_steer_torque) {
            steering = std::min(std::max(steering, -90.0), 90.0);
            steer_target_ = steering + vehicle_params_.steer_offset();
            control_command_.set_steering_target(steer_target_);
          } else {
            steer_torque_target_ =
                steering / 100 * vehicle_params_.max_steer_torque();
            control_command_.set_steering_torque(steer_torque_target_);
          }
          // control_command_.set_steering_torque(steer_pid_cmd_);
          AINFO << "Steering Target = " << steering
                << ", Steer_target_ = " << steer_target_
                << ", Torque Target = " << steer_torque_target_;
          break;
        case KEYCODE_RT1:  // right
        case KEYCODE_RT2:
          steering = control_command_.steering_target();
          steering = GetCommand(steering, -FLAGS_steer_inc_delta);
          if (!FLAGS_enable_teleop_steer_torque) {
            steering = std::min(std::max(steering, -90.0), 90.0);
            steer_target_ = steering + vehicle_params_.steer_offset();
            control_command_.set_steering_target(steer_target_);
          } else {
            steer_torque_target_ =
                steering / 100 * vehicle_params_.max_steer_torque();
            control_command_.set_steering_torque(steer_torque_target_);
          }
          // control_command_.set_steering_torque(steer_pid_cmd_);
          AINFO << "Steering Target = " << steering
                << ", Steer_target_ = " << steer_target_
                << ", Torque Target = " << steer_pid_cmd_;
          break;
        case KEYCODE_PKBK:  // hand brake
          parking_brake = !control_command_.parking_brake();
          control_command_.set_parking_brake(parking_brake);
          AINFO << "Parking Brake Toggled: " << parking_brake;
          break;
        case KEYCODE_ESTOP:
          control_command_.set_brake(50.0);
          AINFO << "Estop Brake : " << control_command_.brake();
          break;
        case KEYCODE_SETT1:  // set throttle
        case KEYCODE_SETT2:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          level = c - KEYCODE_ZERO;
          control_command_.set_throttle(level * 10.0);
          control_command_.set_brake(0.0);
          AINFO << "Throttle = " << control_command_.throttle()
                << ", Brake = " << control_command_.brake();
          break;
        case KEYCODE_Z1:
        case KEYCODE_Z2:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          level = c - KEYCODE_ZERO;
          control_command_.set_ctrl_req_mode(
              (TL::control::CtrlReqMode)level);
          AINFO << "ctrl_req_mode = " << control_command_.ctrl_req_mode();
          break;
        case KEYCODE_SETG1:
        case KEYCODE_SETG2:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          level = c - KEYCODE_ZERO;
          gear = GetGear(level);
          control_command_.set_gear_location(gear);
          AINFO << "Gear set to : " << level;
          break;
        case KEYCODE_SETB1:
        case KEYCODE_SETB2:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          level = c - KEYCODE_ZERO;
          control_command_.set_throttle(0.0);
          control_command_.set_brake(level * 10.0);
          AINFO << "Throttle = " << control_command_.throttle()
                << ", Brake = " << control_command_.brake();
          break;
        case KEYCODE_SETQ1:
        case KEYCODE_SETQ2:
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          static int cnt = 0;
          ++cnt;
          if (cnt > 2) {
            cnt = 0;
          }

          if (cnt == 0) {
            control_command_.mutable_signal()->set_turn_signal(
                VehicleSignal::TURN_NONE);
          } else if (cnt == 1) {
            control_command_.mutable_signal()->set_turn_signal(
                VehicleSignal::TURN_LEFT);
          } else if (cnt == 2) {
            control_command_.mutable_signal()->set_turn_signal(
                VehicleSignal::TURN_RIGHT);
          }

          break;
        case KEYCODE_MODE1:
        case KEYCODE_MODE2:
          // read keyboard again
          if (read(kfd_, &c, 1) < 0) {
            exit(-1);
          }
          level = c - KEYCODE_ZERO;
          GetPadMessage(&pad_msg, level);
          control_command_.mutable_pad_msg()->CopyFrom(pad_msg);
          AINFO << "pad_msg : " << pad_msg.DebugString();
          sleep(1);
          control_command_.clear_pad_msg();
          break;
        case KEYCODE_HELP:
        case KEYCODE_HELP2:
          PrintKeycode();
          break;
        default:
          // printf("%X\n", c);
          break;
      }
      // AINFO << "control command after switch : "
      //       << control_command_.ShortDebugString().c_str();
    }  // keyboard_loop big while
    tcsetattr(kfd_, TCSANOW, &cooked_);
    AINFO << "keyboard_loop thread quited.";
    return;
  }  // end of keyboard loop thread

  ControlCommand& control_command() { return control_command_; }

  const ControlCommand GetControlCmd() {
    Send();
    return control_command_;
  }

  Chassis::GearPosition GetGear(int32_t gear) {
    switch (gear) {
      case 0:
        return Chassis::GEAR_NEUTRAL;
      case 1:
        return Chassis::GEAR_DRIVE;
      case 2:
        return Chassis::GEAR_REVERSE;
      case 3:
        return Chassis::GEAR_PARKING;
      case 4:
        return Chassis::GEAR_LOW;
      case 5:
        return Chassis::GEAR_INVALID;
      case 6:
        return Chassis::GEAR_NONE;
      default:
        return Chassis::GEAR_INVALID;
    }
  }

  void GetPadMessage(PadMessage* pad_msg, int32_t int_action) {
    TL::control::DrivingAction action =
        TL::control::DrivingAction::RESET;
    switch (int_action) {
      case 0:
        action = TL::control::DrivingAction::RESET;
        AINFO << "SET Action RESET";
        break;
      case 1:
        action = TL::control::DrivingAction::START;
        AINFO << "SET Action START";
        break;
      default:
        AINFO << "unknown action: " << int_action << " use default RESET";
        break;
    }
    pad_msg->set_action(action);
    return;
  }

  double GetCommand(double val, double inc) {
    val += inc;
    if (val > 100.0) {
      val = 100.0;
    } else if (val < -100.0) {
      val = -100.0;
    }
    return val;
  }

  void Send() {
    TL::common::util::FillHeader("control", &control_command_);
    // control_command_writer_->Write(control_command_);
    // AWARN << "Control Command send OK:" <<
    // control_command_.ShortDebugString();
  }

  void ResetControlCommand() {
    control_command_.Clear();
    control_command_.set_throttle(0.0);
    control_command_.set_brake(0.0);
    control_command_.set_steering_rate(0.0);
    control_command_.set_steering_target(0.0);
    control_command_.set_parking_brake(false);
    control_command_.set_speed(0.0);
    control_command_.set_acceleration(0.0);
    control_command_.set_reset_model(false);
    control_command_.set_engine_on_off(false);
    control_command_.set_driving_mode(Chassis::COMPLETE_MANUAL);
    control_command_.set_gear_location(Chassis::GEAR_INVALID);
    control_command_.mutable_signal()->set_turn_signal(
        VehicleSignal::TURN_NONE);
  }

  void OnChassis(const Chassis& chassis) { Send(); }
  void SetChassis(const Chassis chassis) { chassis_ = chassis; }

  int32_t Start() {
    if (is_running_) {
      AERROR << "Already running.";
      return -1;
    }
    ACHECK(TL::common::GetProtoFromFile(FLAGS_control_conf_file,
                                            &control_conf_))
        << "Unable to load control conf file: " + FLAGS_control_conf_file;
    mpc_controller_conf_ = control_conf_.mpc_controller_conf();

    is_running_ = true;

    keyboard_thread_.reset(
        new std::thread([this] { KeyboardLoopThreadFunc(); }));
    if (keyboard_thread_ == nullptr) {
      AERROR << "Unable to create can client receiver thread.";
      return -1;
    }
    return 0;
  }

  void Stop() {
    if (is_running_) {
      is_running_ = false;
      if (keyboard_thread_ != nullptr && keyboard_thread_->joinable()) {
        keyboard_thread_->join();
        keyboard_thread_.reset();
        AINFO << "Teleop keyboard stopped [ok].";
      }
    }
  }
  void PIDCalculateFunc() {
    auto chassis = chassis_;
    steer_now_ = chassis.steering_percentage();
    ControlCommand& control_command_ = control_command();
    const ::TL::control::MpcSteerTorqueControllerConf&
        mpc_steer_torque_controller_conf =
            mpc_controller_conf_.mpc_steer_torque_controller_conf();
    double pid_ts = mpc_steer_torque_controller_conf.pid_ts();
    double steer_error_limit =
        mpc_steer_torque_controller_conf.steer_error_limit();
    double torque_error_limit =
        mpc_steer_torque_controller_conf.torque_error_limit();
    double steer_angle_error_limited = 0.0;
    steer_angle_error_limited = ::TL::common::math::Clamp(
        steer_target_ - steer_now_, -steer_error_limit, steer_error_limit);
    double feedforward_k = 0.0;
    double kff_ = 0;
    if (FLAGS_enable_pid_speedsegment_control) {
      double kp_segment = ::TL::common::math::InterpolationOne(
          linear_velocity_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .speed_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .kp_speed_segment());
      double ki_segment = ::TL::common::math::InterpolationOne(
          linear_velocity_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .speed_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .ki_speed_segment());
      double kd_segment = ::TL::common::math::InterpolationOne(
          linear_velocity_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .speed_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .kd_speed_segment());
      double kff_segment = ::TL::common::math::InterpolationOne(
          linear_velocity_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .speed_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .kff_speed_segment());
      kff_ = kff_segment;
      steer_pid_controller_.SetPID_Segment(kp_segment, ki_segment, kd_segment);
    } else {
      double kp_segment = ::TL::common::math::InterpolationOne(
          steer_target_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .steer_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .kp_steer_segment());
      double ki_segment = ::TL::common::math::InterpolationOne(
          steer_target_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .steer_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .ki_steer_segment());
      double kd_segment = ::TL::common::math::InterpolationOne(
          steer_target_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .steer_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .kd_steer_segment());
      double kff_segment = ::TL::common::math::InterpolationOne(
          steer_target_,
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .steer_segment(),
          mpc_steer_torque_controller_conf.steer_segment_pid_conf()
              .kff_steer_segment());
      kff_ = kff_segment;
      steer_pid_controller_.SetPID_Segment(kp_segment, ki_segment, kd_segment);
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
    if (FLAGS_enable_steer_segment_feedforward) {
      steer_torque_feedforward = kff_;
    } else {
      steer_torque_feedforward = steer_target_ > 0.0 ? y : -y;
    }
    double steer_torque =
        steer_pid_controller_.Control(steer_angle_error_limited, pid_ts) +
        steer_torque_feedforward;
    // 右转向
    if (steer_target_ < 0.0) {
      steer_torque = vehicle_params_.steer_2_torque_right() * steer_torque;
    } else {
      steer_torque = vehicle_params_.steer_2_torque_left() * steer_torque;
    }
    static double previous_steer_cmd = 0.0;
    steer_torque = ::TL::common::math::Clamp(
        steer_torque, previous_steer_cmd - 0.06, previous_steer_cmd + 0.06);
    steer_pid_cmd_ = ::TL::common::math::Clamp(
        steer_torque, -torque_error_limit, torque_error_limit);
    // if (std::fabs(steer_now_) > 100.0) {
    //   steer_pid_cmd_ = 0.0;
    // }
    control_command_.set_steering_torque(steer_pid_cmd_);
    previous_steer_cmd = steer_pid_cmd_;

    if (chassis.driving_mode() ==
        ::TL::soc::Chassis::COMPLETE_AUTO_DRIVE) {
      AERROR << "PID: v:" << linear_velocity_
             << " steer_target_: " << steer_target_
             << " steer_now_:" << steer_now_
             << " steer_angle_error_limited:" << steer_angle_error_limited
             << " feedforward " << steer_torque_feedforward
             << " steer_torque:" << steer_torque
             << " steer_cmd:" << steer_pid_cmd_;
    }
  }

  bool IsRunning() const { return is_running_; }

 private:
  std::unique_ptr<std::thread> keyboard_thread_;
  ControlCommand control_command_;
  bool is_running_ = false;
  TL::soc::Chassis chassis_;
  ::TL::control::ControlConf control_conf_;
  ::TL::control::MPCControllerConf mpc_controller_conf_;
  // PID controller
  ::TL::control::PIDController steer_pid_controller_;
  TL::common::VehicleParam vehicle_params_;
  double steer_target_ = 0.0;
  double steer_torque_target_ = 0.0;
  double steer_now_ = 0.0;
  double linear_velocity_ = 0.0;
  double steer_pid_cmd_ = 0.0;
  double steer_calibration_cmd_ = 0.0;
  std::unique_ptr<::TL::control::Interpolation2D>
      mpc_torque_control_interpolation_;
};

}  // namespace Teleop
