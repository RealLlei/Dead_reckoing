/******************************************************************************
 * Copyright 2022 TL Auto Co., Ltd. All rights reserved.
 *****************************************************************************/

/**
 * @file semantics.h
 **/

#pragma once

#include <algorithm>
#include <array>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include "common/math/vec2d.h"
#include "planning/common/game/game_common/idm/idm.h"
#include "proto/common/pnc_point.pb.h"

namespace TL::planning::game_common {

constexpr int kInvalidAgentId = -1;
constexpr double kBigEpsilon = 0.01;
constexpr int kInvalidIndex = -1;
constexpr int kEgoAgentId = -2;

class VehicleParam {
 public:
  inline double width() const { return width_; }

  inline double length() const { return length_; }

  inline double wheel_base() const { return wheel_base_; }

  inline double max_steer_angle() const { return max_steer_angle_; }

  inline double max_longitudinal_acc() const { return max_longitudinal_acc_; }

  inline double max_lateral_acc() const { return max_lateral_acc_; }

  inline double d_cr() const { return d_cr_; }

  inline void set_width(const double val) { width_ = val; }

  inline void set_length(const double val) { length_ = val; }

  inline void set_wheel_base(const double val) { wheel_base_ = val; }

  inline void set_max_steering_angle(const double val) {
    max_steer_angle_ = val;
  }

  inline void set_max_longitudinal_acc(const double val) {
    max_longitudinal_acc_ = val;
  }

  inline void set_max_lateral_acc(const double val) { max_lateral_acc_ = val; }

  inline void set_d_cr(const double val) { d_cr_ = val; }

  void print() const;

  VehicleParam& operator=(const VehicleParam& other) {
    if (this != &other) {
      this->d_cr_ = other.d_cr_;
      this->width_ = other.width_;
      this->length_ = other.length_;
      this->wheel_base_ = other.wheel_base_;
      this->max_steer_angle_ = other.max_steer_angle_;
      this->max_longitudinal_acc_ = other.max_longitudinal_acc_;
      this->max_lateral_acc_ = other.max_lateral_acc_;
      this->d_cr_ = other.d_cr_;
    }
    return *this;
  }

  // 构造函数
  VehicleParam() = default;

  // 析构函数
  ~VehicleParam() = default;

  // 复制构造函数
  VehicleParam(const VehicleParam&) = default;

  // 移动构造函数
  VehicleParam(VehicleParam&&) = default;

  // 移动赋值运算符
  VehicleParam& operator=(VehicleParam&&) = default;

 private:
  double width_ = 1.90;
  double length_ = 4.88;
  double wheel_base_ = 2.85;
  double max_steer_angle_ = 45.0;
  double max_longitudinal_acc_ = 2.0;
  double max_lateral_acc_ = 2.0;
  double d_cr_ = 1.34;  // 几何中心到后轴之间的距离
};

struct State {
  double time_stamp = 0.0;
  common::math::Vec2d position = {0.0, 0.0};
  double angle = 0.0;
  double kappa = 0.0;
  double velocity = 0.0;
  double acceleration = 0.0;
  double steer = 0.0;

  State& operator=(const State& other) {
    if (this != &other) {
      this->time_stamp = other.time_stamp;
      this->position = other.position;
      this->angle = other.angle;
      this->kappa = other.kappa;
      this->velocity = other.velocity;
      this->acceleration = other.acceleration;
      this->steer = other.steer;
    }
    return *this;
  }

  // 构造函数
  State() = default;

  // 析构函数
  ~State() = default;

  // 复制构造函数
  State(const State&) = default;

  // 移动构造函数
  State(State&&) = default;

  // 移动赋值运算符
  State& operator=(State&&) = default;
};

struct FrenetState {
  double time_stamp = 0.0;
  std::array<double, 3> s_state = {};
  std::array<double, 3> l_state = {};

  FrenetState() = default;

  FrenetState(const std::array<double, 3>& s_dt_ddt,
              const std::array<double, 3>& l_ds_dds, const double t = 0)
      : time_stamp(t), s_state(s_dt_ddt), l_state(l_ds_dds) {}

  FrenetState& operator=(const FrenetState& other) {
    if (this != &other) {
      this->time_stamp = other.time_stamp;
      this->l_state = other.l_state;
      this->s_state = other.s_state;
    }
    return *this;
  }

  // 析构函数
  ~FrenetState() = default;

  // 复制构造函数
  FrenetState(const FrenetState&) = default;

  // 移动构造函数
  FrenetState(FrenetState&&) = default;

  // 移动赋值运算符
  FrenetState& operator=(FrenetState&&) = default;
};

class Vehicle {
 public:
  Vehicle() = default;

  Vehicle(const VehicleParam& param, const State& state)
      : param_(param), state_(state) {}

  Vehicle(const int& id, const VehicleParam& param, const State& state)
      : id_(id), param_(param), state_(state) {}

  inline const State& state() const { return state_; }

  inline State& mutable_state() { return state_; }

  inline void set_state(const State& state) { state_ = state; }

  inline const int& id() const { return id_; }

  inline void set_id(const int id) { id_ = id; }

  inline const FrenetState& frenet_state() const { return frenet_state_; }

  inline FrenetState& mutable_frenet_state() { return frenet_state_; }

  inline void set_frenet_state(const FrenetState& frenet_state) {
    frenet_state_ = frenet_state;
  }

  inline void set_is_overside_vehicle(const bool is_overside_vehicle) {
    is_overside_vehicle_ = is_overside_vehicle;
  }

  inline const bool& is_overside_vehicle() const {
    return is_overside_vehicle_;
  }

  inline const VehicleParam& param() const { return param_; }

  inline void set_param(const VehicleParam& param) { param_ = param; }

 private:
  int id_ = kInvalidAgentId;
  std::string type_;
  VehicleParam param_;
  State state_;
  FrenetState frenet_state_ = {};
  bool is_overside_vehicle_ = false;
};

enum class LongitudinalBehavior {
  kMaintain = 0,
  kAccelerate,
  kDecelerate,
  kStopping
};

enum class LateralBehavior {
  kUndefined = 0,
  kLaneKeeping,
  kLaneChangeLeft,
  kLaneChangeRight,
};

enum class LatSimMode { kAlongSide = 0, kCutIn, kMergeIn, kCross };

struct Action {
  LongitudinalBehavior lon = LongitudinalBehavior::kMaintain;
  double t = 0.0;
};

struct EnumClassHash {
  template <typename T>
  size_t operator()(T t) const {
    return static_cast<std::size_t>(t);
  }
};

struct SimParam {
  game_common::IDM::Param idm_param;
  double look_forward_time = 1.5;
  double steer_control_max_lookahead_dist = 250.0;
  double steer_control_min_lookahead_dist = 3.0;
  double max_lat_acceleration_abs = 1.5;
  double max_lat_jerk_abs = 3.0;
  double max_curvature_abs = 0.33;
  double max_lon_acc_jerk = 5.0;
  double max_lon_brake_jerk = 5.0;
  double max_steer_angle_abs = 60.0 / 180.0 * M_PI;
  double max_steer_rate = 0.39;
  bool auto_decelerate_if_lat_failed = true;
  double init_cutin_prob = 0.0;
};

struct ForwardSimAgent {
  ForwardSimAgent() = default;
  LatSimMode lat_mode = LatSimMode::kAlongSide;
  game_common::Vehicle vehicle;
  SimParam sim_param = {};
  LateralBehavior lat_behavior = LateralBehavior::kUndefined;
};

class IndexedMap {
 public:
  bool Has(int key) const { return indexMap_.find(key) != indexMap_.end(); }

  // 插入元素
  void Insert(int key, double value) {
    if (key == kInvalidAgentId) {
      return;
    }
    if (indexMap_.find(key) != indexMap_.end()) {
      return;
    }
    vec_.emplace_back(key, value);
    std::sort(
        vec_.begin(), vec_.end(),
        [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
          return a.second < b.second;
        });
    indexMap_.clear();  // 清空索引映射，因为排序后元素的顺序可能改变
    for (size_t i = 0; i < vec_.size(); ++i) {
      indexMap_[vec_[i].first] = i;
    }
  }

  // 根据键查询索引
  size_t GetIndexByKey(int key) const {
    auto it = indexMap_.find(key);
    if (it != indexMap_.end()) {
      return it->second;
    }
    return kInvalidIndex;  // 表示键不存在
  }

  void Clear() {
    indexMap_.clear();
    vec_.clear();
  }

  // 根据索引查询键和值
  std::pair<int, double> GetElementByIndex(size_t index) const {
    if (index >= 0 && index < vec_.size()) {
      return vec_[index];
    }
    return std::make_pair(-1, -1);  // 表示索引超出范围，返回默认值
  }

  // 根据value查询前一个元素的key和后一个元素的key（如果存在）
  std::pair<int, int> GetPreviousAndNextKeys(int key, double value) const {
    if (vec_.empty()) {
      return {-1, -1};
    }
    bool is_key_in = false;
    if (indexMap_.find(key) != indexMap_.end()) {
      is_key_in = true;
    }
    const auto& it =
        std::lower_bound(vec_.begin(), vec_.end(), value,
                         [](const std::pair<int, double>& a, double b) {
                           return a.second < b + kBigEpsilon;
                         });
    // 获取前一个元素的key
    int pre_key = -1;
    if (it != vec_.begin()) {
      auto it_pre = std::prev(it);
      if (!is_key_in) {
        pre_key = it_pre->first;
      } else {
        if (it_pre != vec_.begin()) {
          auto it_pre_pre = std::prev(it_pre);
          pre_key = it_pre_pre->first;
        }
      }
    }
    // 获取后一个元素的key
    int next_key = -1;
    if (it != vec_.end()) {
      next_key = it->first;
    }

    return std::make_pair(pre_key,
                          next_key);  // 返回前一个元素和后一个元素的key
  }

  // 删除元素
  void Remove(int key) {
    auto it = indexMap_.find(key);
    if (it != indexMap_.end()) {
      vec_.erase(vec_.begin() + static_cast<int>(it->second));
      indexMap_.clear();  // 清空索引映射，因为删除后元素的顺序可能改变
      for (size_t i = 0; i < vec_.size(); ++i) {
        indexMap_[vec_[i].first] = i;
      }
    }
  }

  // 修改元素的值（如果存在）
  void UpdateValue(int key, double value) {
    auto it = indexMap_.find(key);
    if (it != indexMap_.end()) {
      vec_[it->second].second = value;  // 直接修改vector中的值，并刷新排序状态
      std::sort(
          vec_.begin(), vec_.end(),
          [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
            return a.second < b.second;
          });
      indexMap_.clear();
      for (size_t i = 0; i < vec_.size(); ++i) {
        indexMap_[vec_[i].first] = i;
      }
    }
    Insert(key, value);
  }

  const std::map<int, size_t>& GetMapInfo() const { return indexMap_; }

  const std::vector<std::pair<int, double>>& GetVecInfo() const { return vec_; }

 private:
  std::vector<std::pair<int, double>>
      vec_;  // 存储键值对，按value排序的向量，使用自定义的比较函数进行排序
  std::map<int, size_t>
      indexMap_;  // 存储键到索引的映射关系，需要动态更新以保持映射关系准确
};

struct SemanticBehavior {
  std::vector<std::vector<Vehicle>> forward_trajs;
  std::vector<std::vector<LateralBehavior>> forward_behaviors;
  std::vector<std::unordered_map<int, std::vector<Vehicle>>> surround_trajs;
};

}  // namespace TL::planning::game_common
