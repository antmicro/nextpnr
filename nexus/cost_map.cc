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

static void assign_min_entry(delay_t *dst, const delay_t &src)
{
    if (src >= 0) {
        if (*dst < 0) {
            *dst = src;
        } else if (src < *dst) {
            *dst = src;
        }
    }
}

static delay_t penalize(const delay_t &entry, int distance, delay_t penalty)
{
    penalty = std::max(penalty, PENALTY_MIN);
    return entry + distance * penalty * PENALTY_FACTOR;
}

// also known as the L1 norm
static int manhattan_distance(const std::pair<int32_t, int32_t> &a, const std::pair<int32_t, int32_t> &b)
{
    return std::abs(b.first - a.first) + std::abs(b.second - a.second);
}

std::pair<delay_t, int> CostMap::get_nearby_cost_entry(const boost::multi_array<delay_t, 2> &matrix, int cx, int cy,
                                                       const ArcBounds &bounds)
{
#ifdef DEBUG_FILL
    log_info("Filling %d, %d within (%d, %d, %d, %d)\n", cx, cy, bounds.x0, bounds.y0, bounds.x1, bounds.y1);
#endif

    // spiral around (cx, cy) looking for a nearby entry
    bool in_bounds = bounds.contains(cx, cy);
    if (!in_bounds) {
#ifdef DEBUG_FILL
        log_info("Already out of bounds, return!\n");
#endif
        return std::make_pair(-1, 0);
    }
    int n = 0;
    delay_t fill(matrix[cx][cy]);

    while (in_bounds && (fill < 0)) {
        n++;
#ifdef DEBUG_FILL
        log_info("At n = %d\n", n);
#endif
        in_bounds = false;
        delay_t min_entry = -1;
        for (int ox = -n; ox <= n; ox++) {
            int x = cx + ox;
            int oy = n - abs(ox);
            int yp = cy + oy;
            int yn = cy - oy;
#ifdef DEBUG_FILL
            log_info("Testing %d, %d\n", x, yp);
#endif
            if (bounds.contains(x, yp)) {
                assign_min_entry(&min_entry, matrix[x][yp]);
                in_bounds = true;
#ifdef DEBUG_FILL
                log_info("matrix[%d, %d] = %d, min_entry = %d\n", x, yp, matrix[x][yp], min_entry);
#endif
            }
#ifdef DEBUG_FILL
            log_info("Testing %d, %d\n", x, yn);
#endif
            if (bounds.contains(x, yn)) {
                assign_min_entry(&min_entry, matrix[x][yn]);
                in_bounds = true;
#ifdef DEBUG_FILL
                log_info("matrix[%d, %d] = %d, min_entry = %d\n", x, yn, matrix[x][yn], min_entry);
#endif
            }
        }

        if (fill < 0 && min_entry >= 0) {
            fill = min_entry;
        }
    }

    return std::make_pair(fill, n);
}

void CostMap::fill_holes(const Context *ctx, boost::multi_array<delay_t, 2> &matrix,
                         delay_t delay_penalty)
{
    // find missing cost entries and fill them in by copying a nearby cost entry
    std::vector<std::tuple<unsigned, unsigned, delay_t>> missing;
    bool couldnt_fill = false;
    auto shifted_bounds = ArcBounds(0, 0, matrix.shape()[0] - 1, matrix.shape()[1] - 1);
    int max_fill = 0;
    for (unsigned ix = 0; ix < matrix.shape()[0]; ix++) {
        for (unsigned iy = 0; iy < matrix.shape()[1]; iy++) {
            delay_t &cost_entry = matrix[ix][iy];
            if (cost_entry < 0) {
                // maximum search radius
                delay_t filler;
                int distance;
                std::tie(filler, distance) = get_nearby_cost_entry(matrix, ix, iy, shifted_bounds);
                if (filler >= 0) {
                    missing.push_back(std::make_tuple(ix, iy, penalize(filler, distance, delay_penalty)));
                    max_fill = std::max(max_fill, distance);
                } else {
                    couldnt_fill = true;
                }
            }
        }
        if (couldnt_fill) {
            // give up trying to fill an empty matrix
            break;
        }
    }

    // write back the missing entries
    for (auto &xy_entry : missing) {
        matrix[std::get<0>(xy_entry)][std::get<1>(xy_entry)] = std::get<2>(xy_entry);
    }

    if (couldnt_fill) {
        for (unsigned y = 0; y < matrix.shape()[1]; y++) {
            for (unsigned x = 0; x < matrix.shape()[0]; x++) {
                NPNR_ASSERT(matrix[x][y] >= 0);
            }
        }
    }
}

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

delay_t CostMap::get_penalty(const boost::multi_array<delay_t, 2> &matrix) const
{
    delay_t min_delay = std::numeric_limits<delay_t>::max();
    delay_t max_delay = std::numeric_limits<delay_t>::min();

    std::pair<int32_t, int32_t> min_location(0, 0), max_location(0, 0);
    for (unsigned ix = 0; ix < matrix.shape()[0]; ix++) {
        for (unsigned iy = 0; iy < matrix.shape()[1]; iy++) {
            const delay_t &cost_entry = matrix[ix][iy];
            if (cost_entry >= 0) {
                if (cost_entry < min_delay) {
                    min_delay = cost_entry;
                    min_location = std::make_pair(ix, iy);
                }
                if (cost_entry > max_delay) {
                    max_delay = cost_entry;
                    max_location = std::make_pair(ix, iy);
                }
            }
        }
    }

    delay_t delay_penalty =
            (max_delay - min_delay) / static_cast<float>(std::max(1, manhattan_distance(max_location, min_location)));

    return delay_penalty;
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
    std::fill_n(delay_matrix.data.data(), delay_matrix.data.num_elements(), 1000);

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

    fill_holes(ctx, delay_matrix.data, get_penalty(delay_matrix.data));

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
            for (int j = 0; j < height; j++) {
                delay_t d = data[i][j];
                std::string str;
                if (d == std::numeric_limits<delay_t>::max())
                    str = "inf";
                else if (std::to_string(d).size() == 1)
                    str = " ";
                else if (std::to_string(d).size() == 2)
                    str = ".";
                else if (std::to_string(d).size() == 3)
                    str = "*";
                else if (std::to_string(d).size() == 4)
                    str = "o";
                else if (std::to_string(d).size() >= 5)
                    str = "O";

                printf("%s", str.c_str());
            }
            printf("\n");
        }
    }
}

NEXTPNR_NAMESPACE_END
