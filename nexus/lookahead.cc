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

NEXTPNR_NAMESPACE_BEGIN

// also known as the L1 norm
static int manhattan_distance(const std::pair<int32_t, int32_t> &a, const std::pair<int32_t, int32_t> &b)
{
    return std::abs(b.first - a.first) + std::abs(b.second - a.second);
}

void Lookahead::build_lookahead(const Context *ctx, DeterministicRNG *rng)
{
    auto start = std::chrono::high_resolution_clock::now();

    if (ctx->verbose) {
        log_info("Building lookahead, first gathering input and output site wires\n");
    }

    int32_t width = ctx->chip_info->width;
    int32_t height = ctx->chip_info->height;

    int32_t max_dist = manhattan_distance(std::make_pair(0, 0), std::make_pair(width, height));

    for (auto wire_pair : wire_type_id_map) {
        dict<std::pair<int32_t, int32_t>, delay_t> delays;

        for (int32_t i = 0; i < ctx->chip_info->width; i++)
            for (int32_t j = 0; j < ctx->chip_info->height; j++) {
                int32_t dist = manhattan_distance(std::make_pair(0, 0), std::make_pair(i, j));

                int32_t penalty = 1 << (int)((dist * 20) / max_dist);

                delay_t delay = penalty;

                delays.emplace(std::make_pair(i, j), delay);
            }

        log_info("Assigning cost for wire type: %s\n", wire_pair.first.c_str(ctx));
        cost_map.set_cost_map(ctx, wire_pair.second, delays);
    }

    auto end = std::chrono::high_resolution_clock::now();
    if (ctx->verbose) {
        log_info("Done with expansion, dt %02fs\n", std::chrono::duration<float>(end - start).count());
    }

    if (ctx->verbose) {
        log_info("build_lookahead time %.02fs\n", std::chrono::duration<float>(end - start).count());
    }
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

    return true;
}

void Lookahead::to_builder(lookahead_storage::Lookahead::Builder builder) const
{
    cost_map.to_builder(builder.getCostMap());
}

NEXTPNR_NAMESPACE_END
