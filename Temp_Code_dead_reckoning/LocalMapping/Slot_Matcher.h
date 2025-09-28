#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>
#include <functional>
#include <memory>

namespace MAGNA {
namespace Planning {   
namespace LocalMapping {

struct Point2f {
    float x, y;
    Point2f() : x(0), y(0) {}
    Point2f(float x, float y) : x(x), y(y) {}
    
    Point2f operator*(float scalar) const { return Point2f(x * scalar, y * scalar); }
    Point2f operator+(const Point2f& other) const { return Point2f(x + other.x, y + other.y); }
    Point2f operator-(const Point2f& other) const { return Point2f(x - other.x, y - other.y); }
};

struct Vec2f {
    float x, y;
    Vec2f() : x(0), y(0) {}
    Vec2f(float x, float y) : x(x), y(y) {}
    Vec2f(const Point2f& p) : x(p.x), y(p.y) {}
    
    float dot(const Vec2f& other) const {
        return x * other.x + y * other.y;
    }
    
    float length() const {
        return std::sqrt(x * x + y * y);
    }
    
    Vec2f normalized() const {
        float len = length();
        if (len > 0) return Vec2f(x / len, y / len);
        return *this;
    }
    
    Vec2f operator*(float scalar) const { return Vec2f(x * scalar, y * scalar); }
    Vec2f operator+(const Vec2f& other) const { return Vec2f(x + other.x, y + other.y); }
};

struct Detection {
    Point2f center;                   // 车位中心点坐标
    Vec2f direction;                  // 车位方向向量
    std::vector<Point2f> corners;     // 车位的四个角点坐标
    float confidence;                 // 检测置信度 [0, 1]
    int class_id;                     // 类别ID (0:空闲, 1:占用等)
    float width;                      // 车位宽度
    float length;                     // 车位长度
    uint64_t timestamp;               // 检测时间戳
    int sensor_id;                    // 传感器ID
    
    Detection() : confidence(0), class_id(0), width(0), length(0), timestamp(0), sensor_id(0) {}
};

enum class TrackState {
    TEMPORARY,   // 临时状态（未确认）
    CONFIRMED,   // 确认状态
    DELETED      // 已删除
};

struct SlotFeature {
    int id;                           // 稳定ID
    Point2f center;                   // 中心点坐标
    Vec2f direction;                  // 方向向量
    std::vector<Point2f> corners;     // 四个角点
    int age;                          // 存在时间
    int missing_count;                // 连续丢失计数
    TrackState state;                 // 跟踪状态
    float confidence;                 // 置信度
    float width;                      // 车位宽度
    float length;                     // 车位长度
    
    SlotFeature() : id(-1), age(0), missing_count(0), state(TrackState::TEMPORARY), 
                   confidence(0), width(0), length(0) {}
};

// 匈牙利算法实现
class HungarianOptimizer {
private:
    enum class Mark { NONE, PRIME, STAR };
    
    std::vector<std::vector<float>> costs_;
    std::vector<std::vector<Mark>> marks_;
    std::vector<bool> rows_covered_;
    std::vector<bool> cols_covered_;
    std::vector<int> stars_in_col_;
    size_t width_, height_;
    
public:
    std::vector<std::pair<size_t, size_t>> minimize(const std::vector<std::vector<float>>& cost_matrix) {
        width_ = cost_matrix.empty() ? 0 : cost_matrix[0].size();
        height_ = cost_matrix.size();
        
        if (width_ == 0 || height_ == 0) return {};
        
        // 扩展为方阵
        size_t matrix_size = std::max(width_, height_);
        costs_.resize(matrix_size, std::vector<float>(matrix_size, 0));
        marks_.resize(matrix_size, std::vector<Mark>(matrix_size, Mark::NONE));
        
        // 复制成本矩阵
        float max_cost = 0;
        for (size_t i = 0; i < height_; ++i) {
            for (size_t j = 0; j < width_; ++j) {
                costs_[i][j] = cost_matrix[i][j];
                max_cost = std::max(max_cost, cost_matrix[i][j]);
            }
        }
        
        // 填充剩余部分
        for (size_t i = height_; i < matrix_size; ++i) {
            for (size_t j = width_; j < matrix_size; ++j) {
                costs_[i][j] = max_cost;
            }
        }
        
        rows_covered_.assign(matrix_size, false);
        cols_covered_.assign(matrix_size, false);
        stars_in_col_.assign(matrix_size, 0);
        
        reduceRows();
        starZeroes();
        coverStarredZeroes();
        
        while (true) {
            size_t zero_row, zero_col;
            if (findZero(&zero_row, &zero_col)) {
                primeZero(zero_row, zero_col);
                int star_col = findStarInRow(zero_row);
                if (star_col != -1) {
                    coverRow(zero_row);
                    uncoverCol(star_col);
                } else {
                    makeAugmentingPath(zero_row, zero_col);
                    clearCovers();
                    clearPrimes();
                    coverStarredZeroes();
                }
            } else {
                augmentPath();
            }
            
            if (isComplete()) break;
        }
        
        return findAssignments();
    }
    
private:
    void reduceRows() {
        for (size_t i = 0; i < costs_.size(); ++i) {
            float min_val = *std::min_element(costs_[i].begin(), costs_[i].end());
            for (size_t j = 0; j < costs_[i].size(); ++j) {
                costs_[i][j] -= min_val;
            }
        }
    }
    
    void starZeroes() {
        for (size_t i = 0; i < costs_.size(); ++i) {
            for (size_t j = 0; j < costs_[i].size(); ++j) {
                if (costs_[i][j] == 0 && !rows_covered_[i] && !cols_covered_[j]) {
                    marks_[i][j] = Mark::STAR;
                    rows_covered_[i] = true;
                    cols_covered_[j] = true;
                    stars_in_col_[j]++;
                }
            }
        }
        clearCovers();
    }
    
    void coverStarredZeroes() {
        for (size_t j = 0; j < costs_[0].size(); ++j) {
            if (stars_in_col_[j] > 0) {
                cols_covered_[j] = true;
            }
        }
    }
    
    bool findZero(size_t* zero_row, size_t* zero_col) {
        for (size_t i = 0; i < costs_.size(); ++i) {
            if (!rows_covered_[i]) {
                for (size_t j = 0; j < costs_[i].size(); ++j) {
                    if (!cols_covered_[j] && costs_[i][j] == 0) {
                        *zero_row = i;
                        *zero_col = j;
                        return true;
                    }
                }
            }
        }
        return false;
    }
    
    void primeZero(size_t row, size_t col) {
        marks_[row][col] = Mark::PRIME;
    }
    
    int findStarInRow(size_t row) {
        for (size_t j = 0; j < costs_[row].size(); ++j) {
            if (marks_[row][j] == Mark::STAR) {
                return j;
            }
        }
        return -1;
    }
    
    int findStarInCol(size_t col) {
        for (size_t i = 0; i < costs_.size(); ++i) {
            if (marks_[i][col] == Mark::STAR) {
                return i;
            }
        }
        return -1;
    }
    
    void makeAugmentingPath(size_t start_row, size_t start_col) {
        std::vector<std::pair<size_t, size_t>> path;
        path.emplace_back(start_row, start_col);
        
        while (true) {
            int star_col = findStarInRow(path.back().first);
            if (star_col == -1) break;
            path.emplace_back(path.back().first, star_col);
            
            int prime_row = findStarInCol(path.back().second);
            if (prime_row == -1) break;
            path.emplace_back(prime_row, path.back().second);
        }
        
        for (const auto& p : path) {
            if (marks_[p.first][p.second] == Mark::STAR) {
                marks_[p.first][p.second] = Mark::NONE;
                stars_in_col_[p.second]--;
            } else {
                marks_[p.first][p.second] = Mark::STAR;
                stars_in_col_[p.second]++;
            }
        }
    }
    
    void augmentPath() {
        float min_val = std::numeric_limits<float>::max();
        for (size_t i = 0; i < costs_.size(); ++i) {
            if (!rows_covered_[i]) {
                for (size_t j = 0; j < costs_[i].size(); ++j) {
                    if (!cols_covered_[j]) {
                        min_val = std::min(min_val, costs_[i][j]);
                    }
                }
            }
        }
        
        for (size_t i = 0; i < costs_.size(); ++i) {
            for (size_t j = 0; j < costs_[i].size(); ++j) {
                if (rows_covered_[i]) {
                    costs_[i][j] += min_val;
                }
                if (!cols_covered_[j]) {
                    costs_[i][j] -= min_val;
                }
            }
        }
    }
    
    bool isComplete() {
        size_t count = 0;
        for (size_t j = 0; j < cols_covered_.size(); ++j) {
            if (cols_covered_[j]) count++;
        }
        return count >= costs_.size();
    }
    
    void clearCovers() {
        std::fill(rows_covered_.begin(), rows_covered_.end(), false);
        std::fill(cols_covered_.begin(), cols_covered_.end(), false);
    }
    
    void clearPrimes() {
        for (auto& row : marks_) {
            for (auto& mark : row) {
                if (mark == Mark::PRIME) {
                    mark = Mark::NONE;
                }
            }
        }
    }
    
    std::vector<std::pair<size_t, size_t>> findAssignments() {
        std::vector<std::pair<size_t, size_t>> assignments;
        for (size_t i = 0; i < height_; ++i) {
            for (size_t j = 0; j < width_; ++j) {
                if (marks_[i][j] == Mark::STAR) {
                    assignments.emplace_back(i, j);
                    break;
                }
            }
        }
        return assignments;
    }
    
    void coverRow(size_t row) { rows_covered_[row] = true; }
    void uncoverRow(size_t row) { rows_covered_[row] = false; }
    void coverCol(size_t col) { cols_covered_[col] = true; }
    void uncoverCol(size_t col) { cols_covered_[col] = false; }
};

class SlotMap {
public:
    void update(const std::vector<Detection>& detections) {
        // 1. 预测已有跟踪器状态
        predictTracks();
        
        // 2. 级联匹配
        cascadedMatching(detections);
        
        // 3. 清理过期跟踪器
        cleanupTracks();
    }
    
    const std::vector<SlotFeature>& getSlots() const {
        return slots_;
    }
    
    std::vector<SlotFeature> getStableSlots() const {
        std::vector<SlotFeature> result;
        for (const auto& slot : slots_) {
            if (slot.state != TrackState::DELETED && slot.missing_count == 0) {
                result.push_back(slot);
            }
        }
        return result;
    }
    
private:
    std::vector<SlotFeature> slots_;
    int next_id_ = 0;
    int max_slots_ = 16;  // 最多16个车位
    
    // 跟踪参数
    float confirm_threshold_ = 0.7f;
    int confirm_frames_ = 3;
    int delete_after_missing_ = 10;
    float max_association_distance_ = 3.0f;
    
    std::vector<std::vector<float>> buildCostMatrix(const std::vector<SlotFeature>& tracks,
                                                   const std::vector<Detection>& detections) {
        std::vector<std::vector<float>> cost_matrix(tracks.size(), 
                                                   std::vector<float>(detections.size(), 0.0f));
        
        for (size_t i = 0; i < tracks.size(); i++) {
            for (size_t j = 0; j < detections.size(); j++) {
                float cost = 0.0f;
                
                // 1. 中心点距离代价 (40%)
                float dx = tracks[i].center.x - detections[j].center.x;
                float dy = tracks[i].center.y - detections[j].center.y;
                float dist = std::sqrt(dx * dx + dy * dy);
                float dist_cost = std::min(dist / max_association_distance_, 1.0f);
                cost += 0.4f * dist_cost;
                
                // 2. 方向相似性代价 (30%)
                float cos_sim = tracks[i].direction.dot(detections[j].direction);
                float dir_cost = 1.0f - (cos_sim + 1.0f) / 2.0f; // 0到1之间
                cost += 0.3f * dir_cost;
                
                // 3. 形状相似性代价 (30%)
                float track_size = tracks[i].width * tracks[i].length;
                float det_size = detections[j].width * detections[j].length;
                if (track_size > 0 && det_size > 0) {
                    float size_ratio = std::max(track_size, det_size) / std::min(track_size, det_size);
                    float size_cost = std::min((size_ratio - 1.0f) * 2.0f, 1.0f);
                    cost += 0.3f * size_cost;
                }
                
                cost_matrix[i][j] = cost;
            }
        }
        return cost_matrix;
    }
    
    void predictTracks() {
        // 简单预测：假设车位位置基本不变
        for (auto& track : slots_) {
            track.missing_count++;
        }
    }
    
    void cascadedMatching(const std::vector<Detection>& detections) {
        if (detections.empty()) return;
        
        // 按状态和丢失时间分组
        std::vector<std::vector<int>> confirmed_groups(5); // 0-4帧丢失
        std::vector<int> temporary_tracks;
        
        for (size_t i = 0; i < slots_.size(); i++) {
            if (slots_[i].state == TrackState::CONFIRMED) {
                int group = std::min(slots_[i].missing_count, 4);
                if (group < confirmed_groups.size()) {
                    confirmed_groups[group].push_back(i);
                }
            } else if (slots_[i].state == TrackState::TEMPORARY) {
                temporary_tracks.push_back(i);
            }
        }
        
        std::vector<bool> matched_detections(detections.size(), false);
        
        // 第一级：确认态跟踪器匹配（按丢失时间从短到长）
        for (int group = 0; group < confirmed_groups.size(); group++) {
            if (confirmed_groups[group].empty()) continue;
            
            std::vector<SlotFeature> group_tracks;
            for (int idx : confirmed_groups[group]) {
                group_tracks.push_back(slots_[idx]);
            }
            
            auto cost_matrix = buildCostMatrix(group_tracks, detections);
            HungarianOptimizer optimizer;
            auto assignments = optimizer.minimize(cost_matrix);
            
            processAssignments(assignments, confirmed_groups[group], detections, matched_detections);
        }
        
        // 第二级：临时态跟踪器匹配
        if (!temporary_tracks.empty()) {
            std::vector<SlotFeature> temp_tracks;
            for (int idx : temporary_tracks) {
                temp_tracks.push_back(slots_[idx]);
            }
            
            auto cost_matrix = buildCostMatrix(temp_tracks, detections);
            HungarianOptimizer optimizer;
            auto assignments = optimizer.minimize(cost_matrix);
            
            processAssignments(assignments, temporary_tracks, detections, matched_detections);
        }
        
        // 处理未匹配的跟踪器和检测
        handleUnmatchedTracks();
        handleNewDetections(detections, matched_detections);
    }
    
    void processAssignments(const std::vector<std::pair<size_t, size_t>>& assignments,
                          const std::vector<int>& track_indices,
                          const std::vector<Detection>& detections,
                          std::vector<bool>& matched_detections) {
        for (const auto& assignment : assignments) {
            size_t track_idx_in_group = assignment.first;
            size_t det_idx = assignment.second;
            
            if (track_idx_in_group < track_indices.size() && det_idx < detections.size()) {
                int actual_track_idx = track_indices[track_idx_in_group];
                updateMatchedTrack(actual_track_idx, detections[det_idx]);
                matched_detections[det_idx] = true;
            }
        }
    }
    
    void updateMatchedTrack(int track_idx, const Detection& det) {
        SlotFeature& track = slots_[track_idx];
        
        // 指数平滑更新
        float alpha = 0.8f;
        track.center = track.center * alpha + det.center * (1 - alpha);
        track.direction = track.direction * alpha + det.direction * (1 - alpha);
        track.direction = track.direction.normalized();
        
        // 更新其他属性
        track.corners = det.corners;
        track.width = det.width;
        track.length = det.length;
        track.confidence = det.confidence;
        
        track.age++;
        track.missing_count = 0;
        
        // 状态升级
        if (track.state == TrackState::TEMPORARY && track.age >= confirm_frames_) {
            track.state = TrackState::CONFIRMED;
        }
    }
    
    void handleUnmatchedTracks() {
        // 标记长时间丢失的跟踪器为DELETED
        for (auto& track : slots_) {
            if (track.missing_count > delete_after_missing_) {
                track.state = TrackState::DELETED;
            }
        }
    }
    
    void handleNewDetections(const std::vector<Detection>& detections,
                           const std::vector<bool>& matched_detections) {
        for (size_t i = 0; i < detections.size(); ++i) {
            if (!matched_detections[i]) {
                createNewTrack(detections[i]);
            }
        }
    }
    
    void createNewTrack(const Detection& det) {
        // 清理空间（如果需要）
        if (slots_.size() >= max_slots_) {
            auto it = std::remove_if(slots_.begin(), slots_.end(),
                [](const SlotFeature& track) {
                    return track.state == TrackState::DELETED;
                });
            slots_.erase(it, slots_.end());
            
            if (slots_.size() >= max_slots_) {
                // 删除最久未见的
                auto oldest = std::max_element(slots_.begin(), slots_.end(),
                    [](const SlotFeature& a, const SlotFeature& b) {
                        return a.missing_count < b.missing_count;
                    });
                if (oldest != slots_.end()) {
                    *oldest = initializeTrack(det);
                    return;
                }
            }
        }
        
        SlotFeature new_track;
        new_track.id = next_id_++;
        new_track.center = det.center;
        new_track.direction = det.direction;
        new_track.corners = det.corners;
        new_track.width = det.width;
        new_track.length = det.length;
        new_track.age = 1;
        new_track.missing_count = 0;
        new_track.state = TrackState::TEMPORARY;
        new_track.confidence = det.confidence;
        
        slots_.push_back(new_track);
    }
    
    SlotFeature initializeTrack(const Detection& det) {
        SlotFeature track;
        track.id = next_id_++;
        track.center = det.center;
        track.direction = det.direction;
        track.corners = det.corners;
        track.width = det.width;
        track.length = det.length;
        track.age = 1;
        track.missing_count = 0;
        track.state = TrackState::TEMPORARY;
        track.confidence = det.confidence;
        return track;
    }
    
    void cleanupTracks() {
        slots_.erase(std::remove_if(slots_.begin(), slots_.end(),
            [this](const SlotFeature& track) {
                return track.state == TrackState::DELETED ||
                       (track.state == TrackState::TEMPORARY && 
                        track.missing_count > delete_after_missing_ / 2);
            }), slots_.end());
    }
};

} // namespace LocalMapping
} // namespace Planning
} // namespace MAGNA