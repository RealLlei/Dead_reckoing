/**
 * @file semantics.h
 * @author HKUST Aerial Robotics Group
 * @brief
 * @version 0.1
 * @date 2019-03-17
 *
 * @copyright Copyright (c) 2019
 */
#ifndef _CORE_COMMON_INC_BASICS_SEMANTICS_H_
#define _CORE_COMMON_INC_BASICS_SEMANTICS_H_

#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <string>
#include <vector>

#include "planning/common/game/game_common/semantics/semantics.h"


namespace TL {
namespace planning {

/**
 * @brief Semantic behavior is a collection of different behaviors
 * to describe a complex intention.
 * @param ref_lane, may not be a lane existed in the physical
 * world, instead, it may be reconstructed using the physical
 * lanes as well as the discret behavior etc.
 */
// struct SemanticBehavior {
//   // LateralBehavior lat_behavior;
//   // LongitudinalBehavior lon_behavior;
//   // Lane ref_lane;
//   // double actual_desired_velocity{0.0};

//   // 多条自车轨迹，每个轨迹点一个behavior
//   // 每条自车轨迹轨迹对应一组障碍物轨迹，
//   // 目前只发一条自车轨迹
//   vec_E<vec_E<Vehicle>> forward_trajs;
//   vec_E<std::vector<LateralBehavior>> forward_behaviors;
//   vec_E<std::unordered_map<int, vec_E<Vehicle>>> surround_trajs;

//   // State state;

//   // SemanticBehavior() {
//   //   lat_behavior = LateralBehavior::kLaneKeeping;
//   //   lon_behavior = LongitudinalBehavior::kMaintain;
//   // }
//   // SemanticBehavior(const LateralBehavior &beh) : lat_behavior(beh) {}
// };

// 1. 决策保证轨迹无碰撞,考虑限速
// 2. 继续用Vehicle结构
// 3. fallback规划处理
// 4. 仿真时长5s，dt =0.2s
// 5. 规划采用自车forwawrd traj终点，根据决策效果调整策略
// 6. 每个轨迹点一个behavior
// 7. 规划安全检查保留
/**
 * @brief Occupancy grid map
 *
 * @tparam T Data type of the grid map
 * @tparam N_DIM Dimension of the grid map
 */
template <typename T, int N_DIM>
class GridMapND {
 public:
  enum ValType { FREE = 0, OCCUPIED = 100 };

  /**
   * @brief Construct a new GridMapND object
   *
   */
  GridMapND();

  /**
   * @brief Construct a new GridMapND object
   *
   * @param dims_size size of each dimension
   * @param dims_resolution resolution of each dimension
   * @param dims_name name of each dimension
   */
  GridMapND(const std::array<int, N_DIM>& dims_size,
            const std::array<double, N_DIM>& dims_resolution,
            const std::array<std::string, N_DIM>& dims_name);

  inline std::array<int, N_DIM> dims_size() const { return dims_size_; }

  inline int dims_size(const int& dim) const { return dims_size_.at(dim); }

  inline std::array<int, N_DIM> dims_step() const { return dims_step_; }

  inline int dims_step(const int& dim) const { return dims_step_.at(dim); }

  inline std::array<double, N_DIM> dims_resolution() const {
    return dims_resolution_;
  }

  inline double dims_resolution(const int& dim) const {
    return dims_resolution_.at(dim);
  }

  inline std::array<std::string, N_DIM> dims_name() const { return dims_name_; }

  inline std::string dims_name(const int& dim) const {
    return dims_name_.at(dim);
  }

  inline std::array<double, N_DIM> origin() const { return origin_; }

  inline int data_size() const { return data_size_; }

  inline const std::vector<T>* data() const { return &data_; }

  inline T data(const int& i) const { return data_[i]; }

  inline T* get_data_ptr() { return data_.data(); }

  inline const T* data_ptr() const { return data_.data(); }

  inline void set_origin(const std::array<double, N_DIM>& origin) {
    origin_ = origin;
  }

  inline void set_dims_size(const std::array<int, N_DIM>& dims_size) {
    dims_size_ = dims_size;
    SetNDimSteps(dims_size);
    SetDataSize(dims_size);
  }

  inline void set_dims_resolution(
      const std::array<double, N_DIM>& dims_resolution) {
    dims_resolution_ = dims_resolution;
  }

  inline void set_dims_name(const std::array<std::string, N_DIM>& dims_name) {
    dims_name_ = dims_name;
  }

  inline void set_data(const std::vector<T>& in) { data_ = in; }

  /**
   * @brief Set all data in array to 0
   */
  inline void clear_data() { data_ = std::vector<T>(data_size_, 0); }

  /**
   * @brief Fill all data in array using value
   *
   * @param val input value
   */
  inline void fill_data(const T& val) {
    data_ = std::vector<T>(data_size_, val);
  }

  /**
   * @brief Get the Value Using Coordinate
   *
   * @param coord Input coordinate
   * @param val Output value
   * @return bool
   */
  bool GetValueUsingCoordinate(const std::array<int, N_DIM>& coord,
                               T* val) const;

  /**
   * @brief Get the Value Using Global Position
   *
   * @param p_w Input global position
   * @param val Output value
   */
  void GetValueUsingGlobalPosition(const std::array<double, N_DIM>& p_w,
                                   T* val) const;

  /**
   * @brief Check if the input value is equal to the value in map
   *
   * @param p_w Input global position
   * @param val_in Input value
   * @param res Result
   * @return bool
   */
  bool CheckIfEqualUsingGlobalPosition(const std::array<double, N_DIM>& p_w,
                                       const T& val_in, bool* res) const;

  /**
   * @brief Check if the input value is equal to the value in map
   *
   * @param coord Input coordinate
   * @param val_in Input value
   * @param res Result
   * @return bool
   */
  bool CheckIfEqualUsingCoordinate(const std::array<int, N_DIM>& coord,
                                   const T& val_in, bool* res) const;

  /**
   * @brief Set the Value Using Coordinate
   *
   * @param coord Coordinate of the map
   * @param val Input value
   * @return bool
   */
  bool SetValueUsingCoordinate(const std::array<int, N_DIM>& coord,
                               const T& val);

  /**
   * @brief Set the Value Using Global Position
   *
   * @param p_w Global position
   * @param val Input value
   */
  void SetValueUsingGlobalPosition(const std::array<double, N_DIM>& p_w,
                                   const T& val);

  /**
   * @brief Get the Coordinate Using Global Position
   *
   * @param p_w Input global position
   * @return std::array<int, N_DIM> Output coordinate
   */
  std::array<int, N_DIM> GetCoordUsingGlobalPosition(
      const std::array<double, N_DIM>& p_w) const;

  /**
   * @brief Get the Rounded Position Using Global Position object
   *
   * @param p_w Input global position
   * @return std::array<double, N_DIM> Output global position
   */
  std::array<double, N_DIM> GetRoundedPosUsingGlobalPosition(
      const std::array<double, N_DIM>& p_w) const;

  /**
   * @brief Get the Global Position Using Coordinate
   *
   * @param coord Input coordinate
   * @param p_w Output global position
   */
  void GetGlobalPositionUsingCoordinate(const std::array<int, N_DIM>& coord,
                                        std::array<double, N_DIM>* p_w) const;

  /**
   * @brief Get the Coordinate Using Global Metric On Single Dimension
   *
   * @param metric Input global 1-dim position
   * @param i Dimension
   * @param idx Output 1-d coordinate
   */
  void GetCoordUsingGlobalMetricOnSingleDim(const double& metric, const int& i,
                                            int* idx) const;

  /**
   * @brief Get the Global Metric Using Coordinate On Single Dim object
   *
   * @param idx Input 1-d coordinate
   * @param i Dimension
   * @param metric Output 1-d position
   */
  void GetGlobalMetricUsingCoordOnSingleDim(const int& idx, const int& i,
                                            double* metric) const;

  /**
   * @brief Check if the input coordinate is in map range
   *
   * @param coord Input coordinate
   * @return true In range
   * @return false Out of range
   */
  bool CheckCoordInRange(const std::array<int, N_DIM>& coord) const;

  /**
   * @brief Check if the input 1-d coordinate is in map range
   *
   * @param idx Input 1-d coordinate
   * @param i Dimension
   * @return true In range
   * @return false Out of range
   */
  bool CheckCoordInRangeOnSingleDim(const int& idx, const int& i) const;

  /**
   * @brief Get the mono index using N-dim index
   *
   * @param idx Input N-dim index
   * @return int Output 1-dim index
   */
  int GetMonoIdxUsingNDimIdx(const std::array<int, N_DIM>& idx) const;

  /**
   * @brief Get N-dim index using mono index
   *
   * @param idx Input mono index
   * @return std::array<int, N_DIM> Output N-dim index
   */
  std::array<int, N_DIM> GetNDimIdxUsingMonoIdx(const int& idx) const;

 private:
  /**
   * @brief Set the steps of N-dim array
   * @brief E.g. A x-y-z map's steps are {1, x, x*y}
   *
   * @param dims_size Input the size of dimension
   */
  void SetNDimSteps(const std::array<int, N_DIM>& dims_size);

  /**
   * @brief Get the Data Size
   *
   * @param dims_size Input the size of dimension
   */
  void SetDataSize(const std::array<int, N_DIM>& dims_size);

  std::array<int, N_DIM> dims_size_;
  std::array<int, N_DIM> dims_step_;
  std::array<double, N_DIM> dims_resolution_;
  std::array<std::string, N_DIM> dims_name_;
  std::array<double, N_DIM> origin_;

  int data_size_{0};
  std::vector<T> data_;
};

template <int N_DIM>
struct SpatioTemporalSemanticCubeNd {
  double t_lb{}, t_ub{};
  std::array<double, N_DIM> p_lb, p_ub;
  std::array<double, N_DIM> v_lb, v_ub;
  std::array<double, N_DIM> a_lb, a_ub;

  SpatioTemporalSemanticCubeNd() { FillDefaultBounds(); }

  void FillDefaultBounds() {
    // ~ for optimization, infinity bound will cause numerical
    // ~ unstable. So we put some loose bounds by default
    t_lb = 0.0;
    t_ub = 1.0;

    const double default_pos_lb = -1;
    const double default_pos_ub = 1;
    const double default_vel_lb = -50.0;
    const double default_vel_ub = 50.0;
    const double default_acc_lb = -20.0;
    const double default_acc_ub = 20.0;

    p_lb.fill(default_pos_lb);
    v_lb.fill(default_vel_lb);
    a_lb.fill(default_acc_lb);

    p_ub.fill(default_pos_ub);
    v_ub.fill(default_vel_ub);
    a_ub.fill(default_acc_ub);
  }
};

template <typename T, int N_DIM>
struct AxisAlignedCubeNd {
  std::array<T, N_DIM> upper_bound;
  std::array<T, N_DIM> lower_bound;

  AxisAlignedCubeNd() = default;

  AxisAlignedCubeNd(const std::array<T, N_DIM> ub,
                    const std::array<T, N_DIM> lb)
      : upper_bound(ub), lower_bound(lb) {}
};

struct DrivingCube {
  std::vector<std::array<int, 3>> seeds;
  AxisAlignedCubeNd<int, 3> cube;
};

struct DrivingCorridor {
  int id;
  bool is_valid;
  std::vector<DrivingCube> cubes;
};

/**
 * @brief Vehicle info under Frenet-frame
 */
struct FsVehicle {
  game_common::FrenetState frenet_state;
  std::vector<common::math::Vec2d> vertices;
};

}  // namespace planning
}  // namespace TL

#endif  // _CORE_COMMON_INC_BASICS_SEMANTICS_H_
