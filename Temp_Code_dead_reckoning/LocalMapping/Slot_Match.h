#include <vector>

namespace MAGNA {
namespace Planning {   
namespace LocalMapping {

struct Point2f {
    float x, y;
};

struct Vec2f {
    float x, y;
    float dot(const Vec2f& other) const {
        return x * other.x + y * other.y;
    }
};

struct Detection {
    Point2f center;        // 车位中心点坐标
    Vec2f direction;       // 车位方向向量
    std::vector<Point2f> corners; // 车位的四个角点坐标
    float confidence;          // 检测置信度 [0, 1]
    int class_id;              // 类别ID (0:空闲, 1:占用等)
    float width;               // 车位宽度
    float length;              // 车位长度
    
    // 可选的时间戳和来源信息
    uint64_t timestamp;        // 检测时间戳
    int sensor_id;             // 传感器ID
};

struct SlotFeature {
    int id;                    // 稳定ID
    Point2f center;        // 中心点坐标
    float orientation;         // 方向角度
    Vec2f direction;       // 方向向量
    std::vector<Point2f> corners; // 四个角点
    int age;                   // 存在时间
    int missing_count;         // 连续丢失计数
    TrackState state;          // 跟踪状态
    float confidence;          // 置信度
};

enum class TrackState {
    TEMPORARY,   // 临时状态（未确认）
    CONFIRMED,   // 确认状态
    DELETED      // 已删除
};


class SlotMap {

public:
    void update(const std::vector<Detection>& detections);
    const std::vector<SlotFeature>& getSlots() const;
    
private:
    std::vector<SlotFeature> slots_;      // 所有车位
    int next_id_ = 0;                     // 下一个可用ID
    int max_slots_ = 50;                  // 最大车位数量
    
    // 跟踪参数
    float confirm_threshold_ = 0.7f;      // 确认阈值
    int confirm_frames_ = 3;              // 确认所需帧数
    int delete_after_missing_ = 10;       // 删除阈值
    
    std::vector<std::vector<float>> buildCostMatrix(const std::vector<SlotFeature>& tracks,
                           const std::vector<Detection>& detections);
    void cascadedMatching(const std::vector<Detection>& detections);
    void handleUnmatchedTracks();
    void handleNewDetections(const std::vector<Detection>& detections);
};

std::vector<std::vector<float>> SlotMap::buildCostMatrix(const std::vector<SlotFeature>& tracks,
                                const std::vector<Detection>& detections) {
    std::vector<std::vector<float>> cost_matrix(tracks.size(), detections.size(), std::vector<float>(detections.size());
    
    for (size_t i = 0; i < tracks.size(); i++) {
        for (size_t j = 0; j < detections.size(); j++) {
            float cost = 0.0f;
            
            // 1. 中心点距离代价 (40%)
            float dx = tracks[i].center.x - detections[j].center.x;
            float dy = tracks[i].center.y - detections[j].center.y;
            float dist = std::sqrt(dx * dx + dy * dy);
            cost += 0.4f * std::min(dist / 2.0f, 1.0f);
            
            // 2. 方向相似性代价 (30%)
            float cos_sim = tracks[i].direction.dot(detections[j].direction);
            cost += 0.3f * (1.0f - (cos_sim + 1.0f) / 2.0f);
            
            // 3. 形状相似性代价 (30%)
            float size_ratio = std::max(tracks[i].size, detections[j].size) / 
                              std::min(tracks[i].size, detections[j].size);
            cost += 0.3f * std::min(size_ratio - 1.0f, 0.5f);
            
            cost_matrix.at<float>(i, j) = cost;
        }
    }
    return cost_matrix;
}

void SlotMap::cascadedMatching(const std::vector<Detection>& detections) {
    // 按丢失时间分组：确认态 -> 临时态
    std::vector<std::vector<int>> confirmed_tracks;
    std::vector<std::vector<int>> temporary_tracks;
    
    for (size_t i = 0; i < slots_.size(); i++) {
        if (slots_[i].state == CONFIRMED) {
            int group = std::min(slots_[i].missing_count, 4); // 0-4帧
            if (confirmed_tracks.size() <= group) {
                confirmed_tracks.resize(group + 1);
            }
            confirmed_tracks[group].push_back(i);
        } else if (slots_[i].state == TEMPORARY) {
            temporary_tracks.push_back(i);
        }
    }
    
    std::vector<bool> matched_detections(detections.size(), false);
    
    // 第一级：确认态跟踪器匹配（按丢失时间从短到长）
    for (int group = 0; group < confirmed_tracks.size(); group++) {
        if (confirmed_tracks[group].empty()) continue;
        
        // 提取当前组的跟踪器
        std::vector<SlotFeature> group_tracks;
        for (int idx : confirmed_tracks[group]) {
            group_tracks.push_back(slots_[idx]);
        }
        
        // 构建代价矩阵并使用匈牙利算法
        std::vector<std::vector<float>> cost_matrix = buildCostMatrix(group_tracks, detections);
        auto assignments = hungarianAlgorithm(cost_matrix);
        
        // 处理匹配结果
        processAssignments(assignments, group_tracks, detections, matched_detections);
    }
    
    // 第二级：临时态跟踪器匹配
    if (!temporary_tracks.empty()) {
        std::vector<SlotFeature> temp_tracks;
        for (int idx : temporary_tracks) {
            temp_tracks.push_back(slots_[idx]);
        }
        
        std::vector<std::vector<float>> cost_matrix = buildCostMatrix(temp_tracks, detections);
        auto assignments = hungarianAlgorithm(cost_matrix);
        processAssignments(assignments, temp_tracks, detections, matched_detections);
    }
    
    // 处理未匹配的
    handleUnmatchedTracks();
    handleNewDetections(detections, matched_detections);
}

void updateMatchedTrack(int track_idx, const Detection& det) {
    SlotFeature& track = slots_[track_idx];
    
    // 指数平滑更新位置和方向
    track.center = 0.8f * track.center + 0.2f * det.center;
    track.direction = 0.8f * track.direction + 0.2f * det.direction;
    
    track.age++;
    track.missing_count = 0;
    
    // 状态升级：临时 -> 确认
    if (track.state == TEMPORARY && track.age >= confirm_frames_) {
        track.state = CONFIRMED;
    }
}

void createNewTrack(const Detection& det) {
    if (slots_.size() >= max_slots_) {
        // 清理最久未见的跟踪器
        auto oldest = std::min_element(slots_.begin(), slots_.end(),
            [](const SlotFeature& a, const SlotFeature& b) {
                return a.missing_count > b.missing_count;
            });
        *oldest = initializeTrack(det); // 复用槽位
        return;
    }
    
    SlotFeature new_track;
    new_track.id = next_id_++;
    new_track.center = det.center;
    new_track.direction = det.direction;
    new_track.corners = det.corners;
    new_track.age = 1;
    new_track.missing_count = 0;
    new_track.state = TEMPORARY;
    new_track.confidence = det.confidence;
    
    slots_.push_back(new_track);
}

void cleanupTracks() {
    auto it = std::remove_if(slots_.begin(), slots_.end(),
        [this](const SlotFeature& track) {
            // 删除确认态但丢失太久的，或临时态但一直没确认的
            return (track.state == CONFIRMED && 
                    track.missing_count > delete_after_missing_) ||
                   (track.state == TEMPORARY && 
                    track.missing_count > delete_after_missing_ / 2);
        });
    
    slots_.erase(it, slots_.end());
}

struct StableSlot {
    int id;                    // 稳定不变的ID
    Point2f center;        // 平滑后的中心点
    float orientation;         // 平滑后的方向
    TrackState state;          // 当前状态
    int age;                   // 跟踪年龄
    float confidence;          // 综合置信度
};

std::vector<StableSlot> SlotMap::getStableSlots() const {
    std::vector<StableSlot> result;
    
    for (const auto& slot : slots_) {
        if (slot.state != DELETED && slot.missing_count == 0) {
            StableSlot stable_slot;
            stable_slot.id = slot.id;
            stable_slot.center = slot.center;
            stable_slot.orientation = std::atan2(slot.direction[1], slot.direction[0]);
            stable_slot.state = slot.state;
            stable_slot.age = slot.age;
            stable_slot.confidence = slot.confidence;
            
            result.push_back(stable_slot);
        }
    }
    
    return result;
}

}// namespace LocalMapping
}// namespace Planning
}// namespace MAGNA