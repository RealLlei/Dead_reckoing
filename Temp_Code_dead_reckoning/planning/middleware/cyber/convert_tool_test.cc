/***************************************************************************
*
* Copyright (c) 2023 TLauto.com, Inc. All Rights Reserved
*
**************************************************************************/

/**
* @file:   convert_tool_test.cc
* @author: wangshounian(wangshounian@TLauto.com)
* @date:   2023/10/09 13:38:30
* @brief: 
*
**/
#include <gtest/gtest.h>

#include <iostream>
#include <string>
#include "apollo_proto/canbus/chassis.pb.h"
#include "apollo_proto/common/header.pb.h"
#include "apollo_proto/fsm/function_manager.pb.h"
#include "apollo_proto/fsm/trigger_config.pb.h"
#include "apollo_proto/hmi/nns_location.pb.h"
#include "apollo_proto/hmi/nns_router.pb.h"
#include "apollo_proto/map/map.pb.h"
#include "apollo_proto/map/navigation.pb.h"
#include "apollo_proto/perception/perception_obstacle.pb.h"
#include "apollo_proto/perception/traffic_light_detection.pb.h"
#include "apollo_proto/planning/lanemarkers_lane_line.pb.h"
#include "apollo_proto/planning/pad_msg.pb.h"
#include "apollo_proto/planning/planning.pb.h"
#include "apollo_proto/planning/without_lane_follow.pb.h"
#include "apollo_proto/prediction/prediction_obstacle.pb.h"
#include "apollo_proto/routing/routing.pb.h"
#include "cyber/common/log.h"

// #include "common/util/convert_tool.h"
#include "proto/common/header.pb.h"
#include "proto/fsm/function_manager.pb.h"
#include "proto/fsm/trigger_config.pb.h"
#include "proto/hmi/nns_location.pb.h"
#include "proto/hmi/nns_router.pb.h"
#include "proto/localization/localization.pb.h"
#include "proto/map/map.pb.h"
#include "proto/map/navigation.pb.h"
#include "proto/perception/perception_freespace.pb.h"
#include "proto/perception/perception_obstacle.pb.h"
#include "proto/perception/perception_parking_lot.pb.h"
#include "proto/perception/transport_element.pb.h"
#include "proto/planning/lanemarkers_lane_line.pb.h"
#include "proto/planning/pad_msg.pb.h"
#include "proto/planning/planning.pb.h"
#include "proto/planning/without_lane_follow.pb.h"
#include "proto/prediction/prediction_obstacle.pb.h"
#include "proto/routing/routing.pb.h"
#include "proto/soc/chassis.pb.h"

#include "common/util/convert_tool.h"

// RoutingRequest
// RoutingResponse
// MapMsg
// MapStateData
// ADCTrajectory
// Localization (LocalizationEstimate)

namespace TL {
namespace planning {

class DataCreator {
 public:
  static ApolloHeaderPtr CreateApolloHeader() {
    ApolloHeaderPtr rhs = std::make_shared<ApolloHeader>();
    rhs->set_sequence_num(1123456);
    rhs->set_module_name("apollo_module_name");
    rhs->set_timestamp_sec(10);

    return rhs;
  }

  static ApolloMapPtr CreateMap() {
    ApolloMapPtr rhs = std::make_shared<ApolloMap>();
    // header
    rhs->mutable_header()->set_version("version_1");
    // crosswalk
    apollo::hdmap::Crosswalk* crosswalk_ptr = rhs->add_crosswalk();
    crosswalk_ptr->mutable_id()->set_id("map_crosswalk_id");

    // ...

    return rhs;
  }

  static ApolloRoutingRequestPtr CreateRoutingRequest() {
    ApolloRoutingRequestPtr rhs = std::make_shared<ApolloRoutingRequest>();
    return rhs;
  }

  static ApolloRoutingResponsePtr CreateRoutingResponse() {
    ApolloRoutingResponsePtr rhs = std::make_shared<ApolloRoutingResponse>();
    rhs->mutable_header()->CopyFrom(*DataCreator::CreateApolloHeader());
    rhs->add_road()->set_id("road_id_0");
    rhs->mutable_measurement()->set_distance(10);

    return rhs;
  }
};

TEST(ConvertToolTest, ForwardRoutingRequestTest) {
  ApolloRoutingRequestPtr rhs = std::make_shared<ApolloRoutingRequest>();
  rhs->mutable_header()->set_module_name("test_module_name");
  rhs->mutable_header()->set_sequence_num(111);
  rhs->mutable_header()->set_timestamp_sec(123648);

  EXPECT_EQ(rhs->header().module_name(), "test_module_name");
  EXPECT_EQ(rhs->header().sequence_num(), 111);
  EXPECT_EQ(rhs->header().timestamp_sec(), 123648);

  for (int i = 0; i < 10; ++i) {
    apollo::routing::LaneWaypoint* lwp = rhs->add_waypoint();
    lwp->set_id(std::to_string(i));
    lwp->set_s(i);
  }
  for (int i = 10; i < 20; ++i) {
    std::string* s = rhs->add_blacklisted_road();
    *s = std::to_string(i);
  }

  apollo::routing::ParkingInfo pki;
  pki.set_parking_space_id("parking_space_id");
  rhs->mutable_parking_info()->CopyFrom(pki);
  rhs->set_broadcast(true);

  TLRoutingRequestPtr lhs = ConvertTool::ForwardRoutingRequest(rhs);

  // header
  EXPECT_EQ(lhs->header().frame_id(), rhs->header().module_name());
  EXPECT_EQ(lhs->header().seq(), rhs->header().sequence_num());
  EXPECT_EQ(lhs->header().data_stamp(), rhs->header().timestamp_sec());

  // LaneWaypoint
  EXPECT_EQ(rhs->waypoint_size(), 10);
  ASSERT_EQ(lhs->waypoint_size(), rhs->waypoint_size());

  for (int i = 0; i < rhs->waypoint_size(); ++i) {
    EXPECT_EQ(lhs->waypoint(i).id(), rhs->waypoint(i).id());
    EXPECT_EQ(lhs->waypoint(i).s(), rhs->waypoint(i).s());
  }

  // blacklisted_road

  EXPECT_EQ(rhs->blacklisted_road_size(), 10);
  ASSERT_EQ(lhs->blacklisted_road_size(), rhs->blacklisted_road_size());

  for (int i = 0; i < rhs->blacklisted_road_size(); ++i) {
    EXPECT_EQ(lhs->blacklisted_road(i), rhs->blacklisted_road(i));
  }

  // parkinginfo

  const auto& hrz_pki = lhs->parking_info();
  EXPECT_EQ(hrz_pki.parking_space_id(), "parking_space_id");

  // broadcast
  EXPECT_EQ(lhs->broadcast(), true);
}

TEST(ConvertToolTest, ReverseRoutingResponseTest) {
  TLRoutingResponsePtr rhs = std::make_shared<TLRoutingResponse>();
  rhs->mutable_header()->set_frame_id("test_module_name");
  rhs->mutable_header()->set_seq(111);
  rhs->mutable_header()->set_data_stamp(123648);

  TL::routing::RoadSegment rs;
  rs.set_id("test_id");
  rs.passage(10.0);

  TL::routing::Measurement ms;
  ms.set_distance(10.0);
  rhs->measurement();

  TL::routing::RoutingRequest rr;
  rr.mutable_header()->set_frame_id("test_module_name");
  rr.mutable_header()->set_seq(111);
  rr.mutable_header()->set_data_stamp(123648);
  for (int i = 0; i < 10; ++i) {
    TL::routing::LaneWaypoint* lwp = rr.add_waypoint();
    lwp->set_id(std::to_string(i));
    lwp->set_s(i);
  }
  rhs->routing_request();

  rhs->mutable_map_version()->append("test_map_version");
  // rhs->mutable_status()->set_status(TL::common::StatusPb::SUCCESS);
  rhs->set_ehp_reason("test_ehp_reason");
  // rhs->mutable_origin_response().append("test_origin_response");

  ApolloRoutingResponsePtr lhs = ConvertTool::ReverseRoutingResponse(rhs);

  EXPECT_EQ(lhs->header().module_name(), rhs->header().frame_id());
  EXPECT_EQ(lhs->header().sequence_num(), rhs->header().seq());
  EXPECT_EQ(lhs->header().timestamp_sec(), rhs->header().data_stamp());

  EXPECT_EQ(rhs->road_size(), 0);
  // ASSERT_EQ(lhs->road_size(), rhs->road_size());
  // EXPECT_EQ(lhs->road(0).id(), rhs->road(0).id());

  EXPECT_EQ(lhs->measurement().distance(), rhs->measurement().distance());

  // EXPECT_EQ(lhs->routing_request().header().module_name(), "test_module_name");
  // EXPECT_EQ(lhs->routing_request().header().sequence_num(), 111);
  // EXPECT_EQ(lhs->routing_request().header().timestamp_sec(), 123648);
  // EXPECT_EQ(lhs->routing_request().waypoint_size(), 10);
  // for (int i = 0; i < 10; ++i) {
  // EXPECT_EQ(lhs->routing_request().waypoint(i).id(), std::to_string(i));
  // EXPECT_EQ(lhs->routing_request().waypoint(i).s(), i);
  // }

  EXPECT_EQ(lhs->map_version(), "test_map_version");

  // EXPECT_EQ(lhs->status().status(), TL::common::StatusPb::SUCCESS);

  EXPECT_EQ(lhs->ehp_reason(), "test_ehp_reason");

  // EXPECT_EQ(lhs->origin_response().size(), 0);
  // EXPECT_EQ(lhs->origin_response(0), "test_origin_response");
}

TEST(ConvertToolTest, ForwardMapTest) {
  ApolloMapPtr rhs = DataCreator::CreateMap();
  TLMapPtr lhs = ConvertTool::ForwardMap(rhs);

  // header
  ASSERT_EQ(lhs->has_header(), true);
  EXPECT_EQ(lhs->header().version(), "version_1");

  // crosswalk
  ASSERT_EQ(lhs->crosswalk_size(), 1);
  EXPECT_EQ(lhs->crosswalk(0).id().id(), "map_crosswalk_id");
}

TEST(ConvertToolTest, ForwardMapMsgTest) {
  TLMapMsgPtr lhs_empty =
      ConvertTool::ForwardMapMsg(std::make_shared<ApolloMapMsg>());
  EXPECT_EQ(lhs_empty->has_hdmap(), false);
  EXPECT_EQ(lhs_empty->has_localization(), false);
  EXPECT_EQ(lhs_empty->has_routing(), false);
  EXPECT_EQ(lhs_empty->has_header(), true);

  ApolloMapMsgPtr rhs = std::make_shared<ApolloMapMsg>();
  rhs->mutable_hdmap()->CopyFrom(*DataCreator::CreateMap());
  TLMapMsgPtr lhs = ConvertTool::ForwardMapMsg(rhs);

  // hdmap
  {
    // header
    ASSERT_EQ(lhs->hdmap().has_header(), true);
    EXPECT_EQ(lhs->hdmap().header().version(), "version_1");

    // crosswalk
    ASSERT_EQ(lhs->hdmap().crosswalk_size(), 1);
    EXPECT_EQ(lhs->hdmap().crosswalk(0).id().id(), "map_crosswalk_id");
  }

  // routing
}

}  // namespace planning
}  // namespace TL
