#ifndef RAW_ROUTE_DATA_H
#define RAW_ROUTE_DATA_H

#include "extractor/class_data.hpp"
#include "extractor/guidance/turn_instruction.hpp"
#include "extractor/travel_mode.hpp"

#include "engine/phantom_node.hpp"

#include "osrm/coordinate.hpp"

#include "util/guidance/entry_class.hpp"
#include "util/guidance/turn_bearing.hpp"
#include "util/guidance/turn_lanes.hpp"
#include "util/typedefs.hpp"

#include <vector>
#include <algorithm>

namespace osrm
{
namespace engine
{

struct PathData
{
    // id of via node of the turn
    NodeID turn_via_node;
    // name of the street that leads to the turn
    unsigned name_id;
    // segregated edge-based node that leads to the turn
    bool is_segregated;
    // weight that is traveled on the segment until the turn is reached
    // including the turn weight, if one exists
    EdgeWeight weight_until_turn;
    // If this segment immediately preceeds a turn, then duration_of_turn
    // will contain the weight of the turn.  Otherwise it will be 0.
    EdgeWeight weight_of_turn;
    // duration that is traveled on the segment until the turn is reached,
    // including a turn if the segment preceeds one.
    EdgeWeight duration_until_turn;
    // If this segment immediately preceeds a turn, then duration_of_turn
    // will contain the duration of the turn.  Otherwise it will be 0.
    EdgeWeight duration_of_turn;
    // instruction to execute at the turn
    extractor::guidance::TurnInstruction turn_instruction;
    // turn lane data
    util::guidance::LaneTupleIdPair lane_data;
    // travel mode of the street that leads to the turn
    extractor::TravelMode travel_mode : 4;
    // user defined classed of the street that leads to the turn
    extractor::ClassData classes;
    // entry class of the turn, indicating possibility of turns
    util::guidance::EntryClass entry_class;

    // Source of the speed value on this road segment
    DatasourceID datasource_id;

    // bearing (as seen from the intersection) pre-turn
    util::guidance::TurnBearing pre_turn_bearing;
    // bearing (as seen from the intersection) post-turn
    util::guidance::TurnBearing post_turn_bearing;

    // Driving side of the turn
    bool is_left_hand_driving;
};

struct InternalRouteResult
{
    std::vector<std::vector<PathData>> unpacked_path_segments;
    std::vector<PhantomNodes> segment_end_coordinates;
    std::vector<bool> source_traversed_in_reverse;
    std::vector<bool> target_traversed_in_reverse;
    EdgeWeight shortest_path_weight = INVALID_EDGE_WEIGHT;

    bool is_valid() const { return INVALID_EDGE_WEIGHT != shortest_path_weight; }

    bool is_via_leg(const std::size_t leg) const
    {
        return (leg != unpacked_path_segments.size() - 1);
    }

    // Note: includes duration for turns, except for at start and end node.
    EdgeWeight duration() const
    {
        EdgeWeight ret{0};

        for (const auto &leg : unpacked_path_segments)
            for (const auto &segment : leg)
                ret += segment.duration_until_turn;

        return ret;
    }
};

inline void ElongateInternalRouteResult(InternalRouteResult &base_result, InternalRouteResult &new_result, std::size_t ups_index)
{
    BOOST_ASSERT(base_result.is_valid());
    BOOST_ASSERT(new_result.is_valid());
    bool new_leg = base_result.unpacked_path_segments.size() - 1 >= ups_index;
    if (new_leg) base_result.unpacked_path_segments.push_back({});
    auto new_segments_begin = new_result.unpacked_path_segments[ups_index].begin();
    auto new_src_reverse_begin = new_result.source_traversed_in_reverse.begin();
    auto new_trg_reverse_begin = new_result.target_traversed_in_reverse.begin();
    auto new_src_reverse_end = new_result.source_traversed_in_reverse.end();
    auto new_trg_reverse_end = new_result.target_traversed_in_reverse.end();
    auto new_segments_end = new_result.unpacked_path_segments[ups_index].end();
    // if the base result ends where the new one starts, deduplicate
    if (base_result.unpacked_path_segments[ups_index].back().turn_via_node == new_result.unpacked_path_segments[ups_index].front().turn_via_node) // FIXME there must be a better way to make this check
    {
        new_segments_begin++;
        new_src_reverse_begin++; // ?? what is this actually
        new_trg_reverse_begin++;
    }
    std::move(new_segments_begin, new_segments_end, std::back_inserter(base_result.unpacked_path_segments[ups_index]));
    std::move(new_src_reverse_begin, new_src_reverse_end, std::back_inserter(base_result.source_traversed_in_reverse));
    std::move(new_trg_reverse_begin, new_trg_reverse_end, std::back_inserter(base_result.target_traversed_in_reverse));
    base_result.shortest_path_weight = base_result.shortest_path_weight + new_result.shortest_path_weight;
    if (new_leg)
    {
        // if this is a new leg, insert the last phantom_node pair of the new_result into the base_result segment_end_coordinates
        // otherwise, replace the last segment_end_coordinate with the last one in the new_result segment_end_coordinates
    }
}

struct InternalManyRoutesResult
{
    InternalManyRoutesResult() = default;
    InternalManyRoutesResult(InternalRouteResult route) : routes{std::move(route)} {}
    InternalManyRoutesResult(std::vector<InternalRouteResult> routes_) : routes{std::move(routes_)}
    {
    }

    std::vector<InternalRouteResult> routes;
};
}
}

#endif // RAW_ROUTE_DATA_H
