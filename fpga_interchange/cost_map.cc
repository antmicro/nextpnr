/*
 *  nextpnr -- Next Generation Place and Route
 *
 *  Copyright (C) 2021  Symbiflow Authors
 *
 *
 *  Permission to use, copy, modify, and/or distribute this software for any
 *  purpose with or without fee is hereby granted, provided that the above
 *  copyright notice and this permission notice appear in all copies.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 *  WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 *  MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 *  ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 *  WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 *  ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 *  OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 *
 */

#include "cost_map.h"

#include "context.h"
#include "log.h"

NEXTPNR_NAMESPACE_BEGIN

///@brief Factor to adjust the penalty calculation for deltas outside the segment bounding box:
//      factor < 1.0: penalty has less impact on the final returned delay
//      factor > 1.0: penalty has more impact on the final returned delay
static constexpr float PENALTY_FACTOR = 1.f;

///@brief Minimum penalty cost that is added when penalizing a delta outside the segment bounding box.
static constexpr delay_t PENALTY_MIN = 1;

// also known as the L1 norm
static int manhattan_distance(const std::pair<int32_t, int32_t> &a, const std::pair<int32_t, int32_t> &b)
{
    return std::abs(b.first - a.first) + std::abs(b.second - a.second);
}

delay_t CostMap::get_delay(const Context *ctx, WireId src_wire, WireId dst_wire, int32_t wire_type_id) const
{
    const auto &dst_data = ctx->wire_data(dst);
    if (src.tile == 0 && dst_data.name == ID_LOCAL_VCC)
        return 0;
    int src_x = src.tile % chip_info->width, src_y = src.tile / chip_info->width;
    int dst_x = dst.tile % chip_info->width, dst_y = dst.tile / chip_info->width;
    int dist_x = std::abs(src_x - dst_x);
    int dist_y = std::abs(src_y - dst_y);

    auto iter = cost_map_.find(wire_type_id);
    if (iter == cost_map_.end())
        return std::numeric_limits<delay_t>::max();

    const auto &delay_matrix = iter->second;

    // Get the cost entry from the cost map at the deltas values
    auto cost = delay_matrix.data[dist_x][dist_y];
    NPNR_ASSERT(cost >= 0);

    return cost;
}

void CostMap::set_cost_map(const Context *ctx, int32_t wire_type_id,
                           const dict<std::pair<int32_t, int32_t>, delay_t> &delays)
{
    CostMapEntry delay_matrix;

    int max_width = ctx->chip_info->width;
    int max_height = ctx->chip_info->height;

    delay_matrix.data.resize(boost::extents[max_width][max_height]);

    // Fill matrix with sentinel of -1 to know where the holes in the matrix
    // are.
    std::fill_n(delay_matrix.data.data(), delay_matrix.data.num_elements(), std::numeric_limits<delay_t)::max();

    for (const auto &delay_pair : delays) {
        auto &dx_dy = delay_pair.first;
        int32_t dist_x = dx_dy.first;
        int32_t dist_y = dx_dy.second;
        NPNR_ASSERT(dist_x >= 0);
        NPNR_ASSERT(dist_x < max_width);
        NPNR_ASSERT(dist_y >= 0);
        NPNR_ASSERT(dist_y < max_height);

        delay_matrix.data[dist_x][dist_y] = delay_pair.second;
    }

    {
        cost_map_mutex_.lock();
        auto result = cost_map_.emplace(wire_type_id, delay_matrix);
        NPNR_ASSERT(result.second);
        cost_map_mutex_.unlock();
    }
}

void CostMap::from_reader(lookahead_storage::CostMap::Reader reader)
{
    for (auto cost_entry : reader.getCostMap()) {
        TypeWirePair key(cost_entry.getKey());

        auto result = cost_map_.emplace(key, CostMapEntry());
        NPNR_ASSERT(result.second);

        CostMapEntry &entry = result.first->second;
        auto data = cost_entry.getData();
        auto in_iter = data.begin();

        entry.data.resize(boost::extents[cost_entry.getXDim()][cost_entry.getYDim()]);
        if (entry.data.num_elements() != data.size()) {
            log_error("entry.data.num_elements() %zu != data.size() %u", entry.data.num_elements(), data.size());
        }

        delay_t *out = entry.data.origin();
        for (; in_iter != data.end(); ++in_iter, ++out) {
            *out = *in_iter;
        }

        entry.penalty = cost_entry.getPenalty();

        entry.offset.first = cost_entry.getXOffset();
        entry.offset.second = cost_entry.getYOffset();
    }
}

void CostMap::to_builder(lookahead_storage::CostMap::Builder builder) const
{
    auto cost_map = builder.initCostMap(cost_map_.size());
    auto entry_iter = cost_map.begin();
    auto in = cost_map_.begin();
    for (; entry_iter != cost_map.end(); ++entry_iter, ++in) {
        NPNR_ASSERT(in != cost_map_.end());

        in->first.to_builder(entry_iter->getKey());
        const CostMapEntry &entry = in->second;

        auto data = entry_iter->initData(entry.data.num_elements());
        const delay_t *data_in = entry.data.origin();
        for (size_t i = 0; i < entry.data.num_elements(); ++i) {
            data.set(i, data_in[i]);
        }

        entry_iter->setXDim(entry.data.shape()[0]);
        entry_iter->setYDim(entry.data.shape()[1]);
        entry_iter->setXOffset(entry.offset.first);
        entry_iter->setYOffset(entry.offset.second);
        entry_iter->setPenalty(entry.penalty);
    }
}

NEXTPNR_NAMESPACE_END
