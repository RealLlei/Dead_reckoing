// Clean, minimal DeadReckoningCore header used for local compilation without Apollo
#pragma once

#include <cstdint>
#include <memory>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include "../common/kalmanfilter.h"
#include "compat.h"

namespace Magna {
namespace dead_reckoning {

static const unsigned int STATE_NUMBER = 12;
static const double CON_g0 = 9.794645;
static const double MAX_STEER_ANGLE = 475.0;

struct DeadReckoningConfig {
    bool use_imu = false;
    double wheel_radius = 0.35;
    double neta_forward_coef = 50.0;
    double neta_backward_coef = 50.0;
    double rear_wheel_rot_arm = 1.6;
    double default_standstill_value = 0.01;
    bool enable_auto_angle_rate_bias = true;
    int auto_angle_rate_bias_time = 100;
    double default_angle_rate_bias_value = 0.001;
    // missing from original minimal struct but used in .cc
    double gain_yaw_count = 0.0;
    double gain_omg_z_count = 0.0;
};

struct WheelSpeedData {
    uint32_t seq = 0;
    double timestamp = 0.0;
    double fl_wheel_speed = 0.0;
    double fr_wheel_speed = 0.0;
    double rl_wheel_speed = 0.0;
    double rr_wheel_speed = 0.0;
    // alternate field names (some code uses fl_speed etc.)
    double fl_speed = 0.0;
    double fr_speed = 0.0;
    double rl_speed = 0.0;
    double rr_speed = 0.0;
    int rl_dir = 0;
    int rr_dir = 0;
    int fl_dir = 0;
    int fr_dir = 0;
};

struct CounterInfoData {
    uint32_t seq = 0;
    double timestamp = 0.0;
    bool fl_counter_info_isvalid = false;
    bool fr_counter_info_isvalid = false;
    bool rl_counter_info_isvalid = false;
    bool rr_counter_info_isvalid = false;
    double fl_counter_info = 0.0;
    double fr_counter_info = 0.0;
    double rl_counter_info = 0.0;
    double rr_counter_info = 0.0;
    bool is_wheel_cnt_fl_valid() const { return fl_counter_info_isvalid; }
    bool is_wheel_cnt_fr_valid() const { return fr_counter_info_isvalid; }
    bool is_wheel_cnt_rl_valid() const { return rl_counter_info_isvalid; }
    bool is_wheel_cnt_rr_valid() const { return rr_counter_info_isvalid; }
    double wheel_counter_fl() const { return fl_counter_info; }
    double wheel_counter_fr() const { return fr_counter_info; }
    double wheel_counter_rl() const { return rl_counter_info; }
    double wheel_counter_rr() const { return rr_counter_info; }
};

struct ImuData {
    double yaw_rate = 0.0;
    bool has_yaw_rate = false;
    double timestamp = 0.0;
    Eigen::Vector3d linear_acceleration = Eigen::Vector3d::Zero();
    Eigen::Vector3d angular_speed = Eigen::Vector3d::Zero();
};



struct SteerData { double steer_angle = 0.0; double steer_torque = 0.0; };

struct LocalStateData {
    double timestamp = 0.0;
    Eigen::Vector3d pos_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel_w = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_w = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q_i_w = Eigen::Quaterniond::Identity();
    Eigen::Vector3d omg_i = Eigen::Vector3d::Zero();
};

struct ChassisData {
    double timestamp = 0.0;
    bool has_yaw_rate = false;
    double yaw_rate = 0.0;
    bool has_imu_acc = false;
    Eigen::Vector3d imu_acc = Eigen::Vector3d::Zero();
    bool has_wheel_speed = false;
    WheelSpeedData wheel_speed;
    bool has_wheel_counter = false;
    CounterInfoData wheel_counter;
    bool has_steering_percentage() const { return false; }
    double steering_percentage() const { return 0.0; }
    bool has_steering_torque_nm() const { return false; }
    double steering_torque_nm() const { return 0.0; }
};

struct LocalizationOutput {
    double timestamp = 0.0;
    struct Position { double x = 0.0; double y = 0.0; } pos;
    double heading = 0.0;
    double speed = 0.0;
};

class DeadReckoningCore {
 public:
    DeadReckoningCore() = default;
    ~DeadReckoningCore() = default;
    void RunOnce(const std::shared_ptr<const ChassisData>& chassis,
                 const std::shared_ptr<LocalizationOutput>& localization);
    void Init();
    void ResetState();
    void Stop();

 private:
    void SetLocalState();
    bool StateEstimateEkf();
    void PublishLocalization(const std::shared_ptr<LocalizationOutput>& localization);
    void LocalInitialize();
    void LocalStateDynamics(Eigen::VectorXd* dx, const Eigen::VectorXd& x);
    void LocalPredict(const double dt);
    void LocalYawCovUpdate();
    void LocalCarSpeedCorrect();
    void LocalStateUpdate();

    double yaw_counter_ = 0.0;
    double last_timestamp_ = 0.0;
    bool is_imu_bias_init_ = false;
    bool is_forward_ = true;
    bool last_is_forward_ = true;
    bool chassis_is_update_ = false;
    bool wheel_speed_is_valid_ = false;
    bool wheel_counter_is_valid_ = false;
    bool enabled_ = false;
    std::shared_ptr<const ChassisData> chassis_received_;
    WheelSpeedData wheel_speed_;
    WheelSpeedData wheel_counter_speed_;
    CounterInfoData wheel_counter_;
    CounterInfoData last_wheel_counter_;
    ImuData imu_data_;
    SteerData steer_;
    LocalStateData local_state_;
    double last_msg_time_ = 0.0;
    Eigen::VectorXd x_;
    Eigen::MatrixXd P_;
    Eigen::Vector3d pos_;
    Eigen::Quaterniond q_;
    KalmanFilterCorrector car_speed_corrector_;
    KalmanFilterCorrector yaw_cov_corrector_;
    double yaw_l_ = 0.0;
    Eigen::Vector3d gyro_bias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc_bias_ = Eigen::Vector3d::Zero();
    bool is_sensor_ready_ = false;
    bool is_car_standstill_ = false;
    bool is_car_go_straight_ = false;
    double car_standstill_counter_ = 0.0;
    Eigen::Vector3d car_standstill_omg_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d car_standstill_acc_sum_ = Eigen::Vector3d::Zero();
    bool is_local_init_ = false;
    int move_direction_ = 0;
    int get_speed_model_ = 0;
    double neta_forward_coef_ = 50.0;
    double neta_backward_coef_ = 50.0;
    double rear_wheel_rot_arm_ = 1.6;
    DeadReckoningConfig config_;
};

Eigen::MatrixX3d get_cross_mat(const Eigen::Vector3d& w);
Eigen::Vector3d Quaternion2EulerZyx(const Eigen::Quaterniond& q);
Eigen::Vector3d Quaternion2EulerZxy(const Eigen::Quaterniond& q);

}  // namespace dead_reckoning
}  // namespace Magna

// Provide minimal Magna::soc compatibility types expected by the .cc file
namespace Magna {
namespace soc {
using Chassis = Magna::dead_reckoning::ChassisData;
enum WheelSpeed { FORWARD = 0, BACKWARD = 1, UNKNOWN = 2 };
// provide the alternate name expected by the .cc
enum WheelSpeed_WheelSpeedType { WS_FORWARD = 0, WS_BACKWARD = 1, WS_UNKNOWN = 2 };
}  // namespace soc
}  // namespace Magna

// Ensure FLAGS used in the trimmed build are defined
constexpr int FLAGS_auto_angle_rate_bias_time = 100;
constexpr double FLAGS_default_angle_rate_bias_value = 0.001;
