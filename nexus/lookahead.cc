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

#include "lookahead.h"

#include <boost/filesystem.hpp>
#include <boost/safe_numerics/safe_integer.hpp>
#include <capnp/message.h>
#include <capnp/serialize.h>
#include <kj/filesystem.h>
#include <kj/std/iostream.h>
#include <queue>
#include <sstream>
#include <zlib.h>

#include "context.h"
#include "log.h"
#include "scope_lock.h"
#include "flat_wire_map.h"

#include <tbb/parallel_for_each.h>

NEXTPNR_NAMESPACE_BEGIN

static constexpr double kNumberSamplesPercentage = 0.02;
static constexpr int32_t kMaxSamples = 50;
static constexpr int32_t kMaxExploreDist = 100;

struct RoutingNode
{
    WireId wire_to_expand;
    delay_t cost;
    int32_t depth;

    bool operator<(const RoutingNode &other) const { return cost < other.cost; }
};

struct PipAndCost
{
    PipId upstream_pip;
    delay_t cost_from_src;
    int32_t depth;
};

struct DelayStorage
{
    dict<int32_t, dict<std::pair<int32_t, int32_t>, delay_t>> storage;
    int32_t max_explore_depth = 10000;
};

static void update_results(const Context *ctx, const FlatWireMap<PipAndCost> &best_path, int32_t wire_id, WireId src_wire,
                           WireId sink_wire, DelayStorage *storage)
{
    int32_t src_x = src_wire.tile % ctx->chip_info->width;
    int32_t src_y = src_wire.tile / ctx->chip_info->width;

    // Starting from end of result, walk backwards and record the path into
    // the delay storage.
    WireId cursor = sink_wire;
    pool<WireId> seen;
    while (cursor != src_wire) {
        // No loops allowed in routing!
        auto result = seen.emplace(cursor);
        NPNR_ASSERT(result.second);

        int32_t dst_x = cursor.tile % ctx->chip_info->width;
        int32_t dst_y = cursor.tile / ctx->chip_info->width;

        std::pair<int32_t, int32_t> dx_dy;
        dx_dy.first = std::abs(dst_x - src_x);
        dx_dy.second = std::abs(dst_y - src_y);

        const PipAndCost &pip_and_cost = best_path.at(cursor);
        auto &delta_data = storage->storage[wire_id];
        auto result2 = delta_data.emplace(dx_dy, pip_and_cost.cost_from_src);
        if (!result2.second) {
            if (result2.first->second > pip_and_cost.cost_from_src && pip_and_cost.cost_from_src > 0) {
                result2.first->second = pip_and_cost.cost_from_src;
            }
        }

        cursor = ctx->getPipSrcWire(pip_and_cost.upstream_pip);
    }
}

static void expand_routing_graph_from_wire(const Context *ctx, WireId first_wire, DelayStorage *storage, int32_t wire_id)
{
    pool<WireId> seen;
    std::priority_queue<RoutingNode> to_expand;

    int32_t src_x = first_wire.tile % ctx->chip_info->width;
    int32_t src_y = first_wire.tile / ctx->chip_info->width;

    RoutingNode initial;
    initial.cost = 0;
    initial.wire_to_expand = first_wire;
    initial.depth = 0;

    to_expand.push(initial);

    FlatWireMap<PipAndCost> best_path(ctx);

    while (!to_expand.empty()) {
        RoutingNode node = to_expand.top();
        to_expand.pop();

        auto result = seen.emplace(node.wire_to_expand);
        if (!result.second)
            // We've already done an expansion at this wire.
            continue;

        bool has_bel_pin = false;
        for (PipId pip : ctx->getPipsDownhill(node.wire_to_expand)) {

            WireId new_wire = ctx->getPipDstWire(pip);
            if (new_wire == WireId()) {
                continue;
            }

            auto bel_pins = ctx->getWireBelPins(new_wire);
            has_bel_pin = bel_pins.begin() != bel_pins.end();

            RoutingNode next_node;
            next_node.wire_to_expand = new_wire;
            next_node.cost = node.cost + ctx->getPipDelay(pip).maxDelay() + ctx->getWireDelay(new_wire).maxDelay();
            next_node.depth = node.depth + 1;

            // Record best path.
            PipAndCost pip_and_cost;
            pip_and_cost.upstream_pip = pip;
            pip_and_cost.cost_from_src = next_node.cost;
            pip_and_cost.depth = next_node.depth;
            auto result = best_path.emplace(new_wire, pip_and_cost);
            bool is_best_path = true;
            if (!result.second) {
                if (result.first.second->cost_from_src > next_node.cost) {
                    result.first.second->cost_from_src = next_node.cost;
                    result.first.second->upstream_pip = pip;
                    result.first.second->depth = next_node.depth;
                } else {
                    is_best_path = false;
                }
            }

            Loc dst = ctx->getPipLocation(pip);
            int32_t dst_x = dst.x;
            int32_t dst_y = dst.y;
            if (is_best_path && std::abs(dst_x - src_x) < kMaxExploreDist &&
                std::abs(dst_y - src_y) < kMaxExploreDist && next_node.depth < storage->max_explore_depth) {
                to_expand.push(next_node);
            }
        }

        if (has_bel_pin)
            update_results(ctx, best_path, wire_id, first_wire, node.wire_to_expand, storage);
    }
}

static void expand_wire_type_parallel(const Context *ctx, int32_t wire_id, std::vector<WireId> &wires, DeterministicRNG *rng, DelayStorage *storage)
{
    int wire_count = std::min((int)(wires.size() / 100 * kNumberSamplesPercentage), kMaxSamples);
    log_info("Computing %s costs using %d samples\n", ctx->getWireType(wires[0]).c_str(ctx), wire_count);

    for (size_t count = 0; count < wire_count; count++) {
        WireId wire = wires[rng->rng(wires.size())];

        log_info("  - Exapnding %s (tile: %d - index: %d)\n", ctx->getWireType(wire).c_str(ctx), wire.tile, wire.index);
        expand_routing_graph_from_wire(ctx, wire, storage, wire_id);
    }
}

void Lookahead::build_lookahead(const Context *ctx, DeterministicRNG *rng)
{
    auto start = std::chrono::high_resolution_clock::now();

    log_info("Building lookahead, first gathering input and output site wires\n");

    std::vector<std::pair<int32_t, int32_t>> xys;

    dict<int32_t, std::vector<WireId>> wires_of_type;
    DelayStorage storage;

    for (auto wire_pair : wire_type_id_map) {
        auto wire_id = wire_pair.second;

        storage.storage.emplace(wire_id, dict<std::pair<int32_t, int32_t>, delay_t>());

        for (auto wire : ctx->getWires()) {
            if (wire_pair.first != ctx->getWireType(wire))
                continue;

            auto result = wires_of_type.emplace(wire_id, std::vector<WireId>{wire});

            if (!result.second)
                result.first->second.push_back(wire);
        }
    }

    {
        tbb::parallel_for_each(wires_of_type, [&](std::pair<int32_t, std::vector<WireId>> wires) {
            expand_wire_type_parallel(ctx, wires.first, wires.second, rng, &storage);
        });
    }

    for (auto wire_pair : wire_type_id_map) {
        int32_t wire_id = wire_pair.second;
        bool has_storage = storage.storage.count(wire_pair.second);
        NPNR_ASSERT(has_storage);

        log_info("Delays for wire type %s\n", wire_pair.first.c_str(ctx));
        for (auto delay_pair : storage.storage[wire_id])
            log_info("  - %d (%d, %d)\n", delay_pair.second, delay_pair.first.first, delay_pair.first.second);

        cost_map.set_cost_map(ctx, wire_id, storage.storage[wire_id]);
    }

    auto end = std::chrono::high_resolution_clock::now();

    cost_map.print();
    log_info("build_lookahead time %.02fs\n", std::chrono::duration<float>(end - start).count());
}

constexpr static bool kUseGzipForLookahead = false;

static void write_message(::capnp::MallocMessageBuilder &message, const std::string &filename)
{
    kj::Array<capnp::word> words = messageToFlatArray(message);
    kj::ArrayPtr<kj::byte> bytes = words.asBytes();

    boost::filesystem::path temp = boost::filesystem::unique_path();
    log_info("Writing tempfile to %s\n", temp.c_str());

    if (kUseGzipForLookahead) {
        gzFile file = gzopen(temp.c_str(), "w");
        NPNR_ASSERT(file != Z_NULL);

        size_t bytes_written = 0;
        int result;
        while (bytes_written < bytes.size()) {
            size_t bytes_remaining = bytes.size() - bytes_written;
            size_t bytes_to_write = bytes_remaining;
            if (bytes_to_write >= std::numeric_limits<int>::max()) {
                bytes_to_write = std::numeric_limits<int>::max();
            }
            result = gzwrite(file, &bytes[0] + bytes_written, bytes_to_write);
            if (result < 0) {
                break;
            }

            bytes_written += result;
        }

        int error;
        std::string error_str;
        if (result < 0) {
            error_str.assign(gzerror(file, &error));
        }
        NPNR_ASSERT(gzclose(file) == Z_OK);
        if (bytes_written != bytes.size()) {
            // Remove failed writes before reporting error.
            boost::filesystem::remove(temp);
        }

        if (result < 0) {
            log_error("Failed to write lookahead, error from gzip %s\n", error_str.c_str());
        } else {
            if (bytes_written != bytes.size()) {
                log_error("Failed to write lookahead, wrote %d bytes, had %zu bytes\n", result, bytes.size());
            } else {
                // Written, move file into place
                boost::filesystem::rename(temp, filename);
            }
        }
    } else {
        {
            kj::Own<kj::Filesystem> fs = kj::newDiskFilesystem();

            auto path = kj::Path::parse(temp);
            auto file = fs->getCurrent().openFile(path, kj::WriteMode::CREATE);
            file->writeAll(bytes);
        }

        boost::filesystem::rename(temp, filename);
    }
}

bool Lookahead::read_lookahead(const std::string &filename, const Context *ctx)
{
    capnp::ReaderOptions reader_options;
    reader_options.traversalLimitInWords = 32llu * 1024llu * 1024llu * 1024llu;

    if (kUseGzipForLookahead) {
        gzFile file = gzopen(filename.c_str(), "r");
        if (file == Z_NULL) {
            return false;
        }

        std::vector<uint8_t> output_data;
        output_data.resize(4096);
        std::stringstream sstream(std::ios_base::in | std::ios_base::out | std::ios_base::binary);
        while (true) {
            int ret = gzread(file, output_data.data(), output_data.size());
            NPNR_ASSERT(ret >= 0);
            if (ret > 0) {
                sstream.write((const char *)output_data.data(), ret);
                NPNR_ASSERT(sstream);
            } else {
                NPNR_ASSERT(ret == 0);
                int error;
                gzerror(file, &error);
                NPNR_ASSERT(error == Z_OK);
                break;
            }
        }

        NPNR_ASSERT(gzclose(file) == Z_OK);

        sstream.seekg(0);
        kj::std::StdInputStream istream(sstream);
        capnp::InputStreamMessageReader message_reader(istream, reader_options);

        lookahead_storage::Lookahead::Reader lookahead = message_reader.getRoot<lookahead_storage::Lookahead>();
        return from_reader(lookahead, ctx);
    } else {
        boost::iostreams::mapped_file_source file;
        try {
            file.open(filename.c_str());
        } catch (std::ios_base::failure &fail) {
            return false;
        }

        if (!file.is_open()) {
            return false;
        }

        const char *data = reinterpret_cast<const char *>(file.data());
        const kj::ArrayPtr<const ::capnp::word> words =
                kj::arrayPtr(reinterpret_cast<const ::capnp::word *>(data), file.size() / sizeof(::capnp::word));
        ::capnp::FlatArrayMessageReader reader(words, reader_options);
        lookahead_storage::Lookahead::Reader lookahead = reader.getRoot<lookahead_storage::Lookahead>();
        return from_reader(lookahead, ctx);
    }
}

void Lookahead::write_lookahead(const std::string &file) const
{
    ::capnp::MallocMessageBuilder message;

    lookahead_storage::Lookahead::Builder lookahead = message.initRoot<lookahead_storage::Lookahead>();
    to_builder(lookahead);
    write_message(message, file);
}

void Lookahead::init(const Context *ctx, DeterministicRNG *rng)
{
    std::string lookahead_filename;
    if (kUseGzipForLookahead) {
        lookahead_filename = ctx->device + ".lookahead.tgz";
    } else {
        lookahead_filename = ctx->device + ".lookahead";
    }

    int32_t wire_id = 0;
    for (auto wire : ctx->getWires())
        wire_type_id_map.emplace(ctx->getWireType(wire), wire_id++);

    if (!read_lookahead(lookahead_filename, ctx)) {
        build_lookahead(ctx, rng);
        write_lookahead(lookahead_filename);
    }
}

delay_t Lookahead::estimateDelay(const Context *ctx, WireId src, WireId dst) const
{
    auto id = wire_type_id_map.at(ctx->getWireType(src));
    return cost_map.get_delay(ctx, src, dst, id);
}

bool Lookahead::from_reader(lookahead_storage::Lookahead::Reader reader, const Context* ctx)
{
    cost_map.from_reader(reader.getCostMap(), ctx);
    cost_map.print();

    return true;
}

void Lookahead::to_builder(lookahead_storage::Lookahead::Builder builder) const
{
    cost_map.to_builder(builder.getCostMap());
}

NEXTPNR_NAMESPACE_END
