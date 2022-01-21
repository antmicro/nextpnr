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

#ifndef LOOKAHEAD_H
#define LOOKAHEAD_H

#include <algorithm>
#include <vector>

#include "cost_map.h"
#include "deterministic_rng.h"
#include "lookahead.capnp.h"
#include "nextpnr_namespaces.h"

NEXTPNR_NAMESPACE_BEGIN

// Lookahead is a routing graph generic lookahead builder and evaluator.
//
// The lookahead data model is structured into 3 parts:
//  - Output site wires to routing network cost
//  - Routing network point to point cost
//  - Routing network cost to input site wires
//
//  If the lookahead is invoked from a routing wire to a routing wire, only
//  the point to point cost is used.
//
//  If the lookahead is invoked from an output site wire to a routing wire,
//  the point to point cost is computed using the cheapest output routing wire
//  from the current site wire and then returned cost is the sum of the output
//  cost plus the point to point routing network cost.
//
//  If the lookahead is invoked from a routing wire to an input site wire,
//  then the cost is the point to point routing cost to the cheapest input
//  routing wire plus the input routing cost.
//
//  If the lookahead is invoked from an output site wire to an input site wire,
//  then cost is the sum of each of the 3 parts.
struct Lookahead
{
    public:
        void init(const Context *, DeterministicRNG *rng);
        void build_lookahead(const Context *, DeterministicRNG *rng);

        bool read_lookahead(const std::string &file, const Context *ctx);
        void write_lookahead(const std::string &file) const;
        bool from_reader(lookahead_storage::Lookahead::Reader reader, const Context *ctx);
        void to_builder(lookahead_storage::Lookahead::Builder builder) const;

        delay_t estimateDelay(const Context *, WireId src, WireId dst) const;


    private:
        CostMap cost_map;

        dict<IdString, int32_t> wire_type_id_map;
};

NEXTPNR_NAMESPACE_END

#endif /* LOOKAHEAD_H */
