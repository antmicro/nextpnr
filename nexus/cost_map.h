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

#ifndef COST_MAP_H
#define COST_MAP_H

#include <boost/multi_array.hpp>
#include <mutex>

#include "lookahead.capnp.h"
#include "nextpnr_namespaces.h"
#include "nextpnr_types.h"

NEXTPNR_NAMESPACE_BEGIN

struct Context;

class CostMap {
  public:
    delay_t get_delay(const Context *ctx, WireId src, WireId dst, int32_t wire_type_id) const;
    void set_cost_map(const Context *ctx, int32_t wire_type_id,
                      const dict<std::pair<int32_t, int32_t>, delay_t> &delays);

    void from_reader(lookahead_storage::CostMap::Reader reader, const Context *ctx);
    void to_builder(lookahead_storage::CostMap::Builder builder) const;

    void print();

  private:
    struct CostMapEntry {
        boost::multi_array<delay_t, 2> data;
    };

    std::mutex cost_map_mutex_;
    dict<int32_t, CostMapEntry> cost_map_;
};

NEXTPNR_NAMESPACE_END

#endif /* COST_MAP_H */
