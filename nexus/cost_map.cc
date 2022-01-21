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

delay_t CostMap::get_delay(const Context *ctx, WireId src_wire, WireId dst_wire, int32_t wire_type_id) const
{
    const auto &dst_data = ctx->wire_data(dst_wire);
    if (src_wire.tile == 0 && dst_data.name == ID_LOCAL_VCC)
        return 0;
    int src_x = src_wire.tile % ctx->chip_info->width, src_y = src_wire.tile / ctx->chip_info->width;
    int dst_x = dst_wire.tile % ctx->chip_info->width, dst_y = dst_wire.tile / ctx->chip_info->width;
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
    std::fill_n(delay_matrix.data.data(), delay_matrix.data.num_elements(), std::numeric_limits<delay_t>::min());

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

void CostMap::from_reader(lookahead_storage::CostMap::Reader reader, const Context *ctx)
{
    for (auto cost_entry : reader.getCostMap()) {
        int32_t key = cost_entry.getKey();

        auto result = cost_map_.emplace(key, CostMapEntry());
        NPNR_ASSERT(result.second);

        CostMapEntry &entry = result.first->second;
        auto data = cost_entry.getData();
        auto in_iter = data.begin();

        entry.data.resize(boost::extents[ctx->chip_info->width][ctx->chip_info->height]);
        if (entry.data.num_elements() != data.size()) {
            log_error("entry.data.num_elements() %zu != data.size() %u", entry.data.num_elements(), data.size());
        }

        delay_t *out = entry.data.origin();
        for (; in_iter != data.end(); ++in_iter, ++out) {
            *out = *in_iter;
        }
    }
}

void CostMap::to_builder(lookahead_storage::CostMap::Builder builder) const
{
    auto cost_map = builder.initCostMap(cost_map_.size());
    auto entry_iter = cost_map.begin();
    auto in = cost_map_.begin();
    for (; entry_iter != cost_map.end(); ++entry_iter, ++in) {
        NPNR_ASSERT(in != cost_map_.end());

        entry_iter->setKey(in->first);
        const CostMapEntry &entry = in->second;

        auto data = entry_iter->initData(entry.data.num_elements());
        const delay_t *data_in = entry.data.origin();
        for (size_t i = 0; i < entry.data.num_elements(); ++i) {
            data.set(i, data_in[i]);
        }
    }
}

void CostMap::print() {
    for (auto entry : cost_map_) {
        log_info("Cost for wire type id: %d\n", entry.first);

        auto &data = entry.second.data;

        int width = data.shape()[0];
        int height = data.shape()[1];

        for (int i = 0; i < width; i++) {
            for (int j = 0; j < height; j++)
                printf("%d ", data[i][j]);
            printf("\n");
        }
    }
}

NEXTPNR_NAMESPACE_END
