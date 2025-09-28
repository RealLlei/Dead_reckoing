
//  Copyright (c) TL Technologies Co., Ltd. 2019-2021. All rights reserved.

#pragma once
#include <memory>

#include "map/hdmap/hdmap.h"
#include "planning/localview/local_view.h"

namespace TL {
namespace planning {

class BuildLocalViewEvent {};

//
class NoMapfileExistEvent {};

// 由感知转到地图模式，带有强制使用高精地图标志-01
class PerceptionToMapWithFlagEvent {};

// 正常由地图到感知模式-02
class MapToPerceptionEvent {};

// 感知到fused模式,正常的切换-04
class PerceptionToFusionEvent {};

// 融合到感知-08
class FusionToPerceptionEvent {};

// initial to map
class InitialToMapEvent {};

class InitialToPerceptionEvent {};

class InitialToFusionEvent {};

class InitialToParkingEvent {};

class InitialToHistoryTraceEvent {};

class InitialToHDMapAVPEvent {};

class ParkingToInitialEvent {};

class HistoryTraceToInitialEvent {};

class HDMapAVPToInitialEvent {};

class NnpToInitialEvent {};

}  // namespace planning
}  // namespace TL
