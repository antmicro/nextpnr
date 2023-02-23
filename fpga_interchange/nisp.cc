#include "nisp.h"

#include <algorithm>
#include <cassert>
#include <forward_list>
#include <fstream>
#include <iostream>
#include <json11.hpp>
#include <unordered_set>
#include <set>
#include <stdlib.h>
#include <string>
#include <regex>

#include "arch.h"
#include "basectx.h"
#include "chipdb.h"
#include "context.h"
#include "log.h"
#include "vivado.h"
#include "vivado_impl.h"

#include "color_log.h"

NEXTPNR_NAMESPACE_BEGIN

#define NISP_VERBOSE

namespace nisp {

/* ========================================================================= *
 *                               NispHandler                                 *
 * ========================================================================= */

/**
 * @brief Parses the JSON file produced by NISP and saves data inside the NispHandler
 *
 * Here is the JSON structure
 * "SITE_NAME": {
 *   "pin_to_pin_routing": {
 *     "BEL0.PIN0->BEL1.PIN1": {
 *       "requires": [
 *         [
 *           {
 *             "NegVar": {
 *               "Port": "BELx.PINx"
 *             }
 *           }
 *           <...>
 *         ]
 *       ],
 *       "implies": [
 *         [
 *           {
 *             "Var": {
 *               "Port": "I31.I31"
 *             }
 *           }
 *           <...>
 *         ]
 *       ]
 *     },
 *   },
 * },
 *
 * @param path - path to the JSON file produced by NISP
 */
void NispHandler::fillLookup(std::string path)
{
    auto parseBelPinName = [&](const std::string& belpin) -> belpin_name_t {
        // FIXME: Check if the string has correct syntax
        auto bel = belpin.substr(0, belpin.find('.'));
        auto pin = belpin.substr(bel.length() + 1);

        return std::make_pair(this->ctx->id(bel), this->ctx->id(pin));
    };

    auto parseConnStr = [&](const std::string& conn) {
        const std::string delimiter = "->";

        // FIXME: Check if the string has correct syntax
        auto src_name = conn.substr(0, conn.find(delimiter));
        auto dst_name = conn.substr(src_name.length() + delimiter.length());

        return std::make_pair(
            parseBelPinName(src_name),
            parseBelPinName(dst_name)
        );
    };

    // ................................


    std::ifstream file(path);
    if (!file) {
        log_error("Unable to open routing lookup (%s)\n", path.c_str());
        return;
    }
    std::string json_str((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
    std::string error;
    auto root = json11::Json::parse(json_str, error, json11::JsonParse::COMMENTS);
    if (root.is_null()) {
        log_error("Unable to parse routing lookup json");
        return;
    }

    IdString VCC_const = this->ctx->id("$VCC");
    IdString GND_const = this->ctx->id("$GND");


    auto root_map = root.object_items();
    for (auto root_it = root_map.begin(); root_it != root_map.end(); ++root_it) {
        site_name_t site_type_name = ctx->id(root_it->first);
        auto &json_data = root_it->second;

        auto& st_data = this->site_type_data[site_type_name];

        auto &pin_to_pin_routing = json_data["pin_to_pin_routing"];
        auto p2p_map = pin_to_pin_routing.object_items();

        // Count tags and map them to indices
        size_t current_tag_idx = 0;
        dict<belpin_name_t, size_t> tag_to_idx;
        for (auto p2p_it = p2p_map.begin(); p2p_it != p2p_map.end(); p2p_it++) {
            auto implies = p2p_it->second["implies"].array_items();
            auto requires = p2p_it->second["requires"].array_items();

            for (const auto &r : requires) {
                for (const auto &rr : r.array_items()) {
                    belpin_name_t tag = parseBelPinName(rr["NegVar"]["Port"].string_value());
                    if (tag_to_idx.emplace(tag, current_tag_idx).second)
                        current_tag_idx++;
                }
            }
            for (const auto &r : implies) {
                for (const auto &rr : r.array_items()) {
                    belpin_name_t tag = parseBelPinName(rr["Var"]["Port"].string_value());
                    if (tag_to_idx.emplace(tag, current_tag_idx).second)
                        current_tag_idx++;
                }
            }

        }

#ifdef ENABLE_NISP_DUMPS
        site_tag_idx_map[site_type_name] = tag_to_idx;
#endif

        st_data.tag_cnt = current_tag_idx;

        nisp_log("NISP: Loading lookup for site type '%s' (%zu bits)\n",
                    root_it->first.c_str(), st_data.tag_cnt);

        // Create p2p_routability lookup
        for (auto p2p_it = p2p_map.begin(); p2p_it != p2p_map.end(); p2p_it++) {
            auto implies = p2p_it->second["implies"].array_items();
            auto requires = p2p_it->second["requires"].array_items();

            // get source and sink names as BEL, BEL pin name pairs
            auto connData = parseConnStr(p2p_it->first);
            auto src_name = connData.first;
            auto sink_name = connData.second;

            if (requires.size() != implies.size()) {
                log_error("NISP: (data import) \"requires\" array length does not match "
                          "\"implies\" array length. Did you forget to run NISP with "
                          "`--no-formula-opt` option enabled?\n");
            }

            auto& rt_info = st_data.p2p_routability[src_name][sink_name];

            for (size_t i = 0; i < requires.size(); i++) {
                const auto& implies_tag_set = implies[i];

                auto p2prt = PinPairRoutability(st_data.tag_cnt);

                for (const auto &rr : requires[i].array_items()) {
                    belpin_name_t tag = parseBelPinName(rr["NegVar"]["Port"].string_value());
                    p2prt.requires.setBit(tag_to_idx.at(tag), true);
                }

                for (const auto &rr : implies[i].array_items()) {
                    belpin_name_t tag = parseBelPinName(rr["Var"]["Port"].string_value());
                    p2prt.implies.setBit(tag_to_idx.at(tag), true);
                }

                rt_info.emplace_back(std::move(p2prt));
            }
        }

        // parse "out_of_site_sources"

        auto &out_of_site_sources = json_data["out_of_site_sources"];
        auto out_src_map = out_of_site_sources.object_items();

        for (auto out_src_it = out_src_map.begin(); out_src_it != out_src_map.end(); out_src_it++) {
            belpin_name_t out_port = parseBelPinName(out_src_it->first);

            auto in_ports = out_src_it->second.array_items();
            for (auto const &in_port_it : in_ports) {
                belpin_name_t in_port = parseBelPinName(in_port_it.string_value());

                // Dedicated constant sources need some special treatment
                if (in_port.second == VCC_const) {
                    st_data.oos_vcc = in_port;
                    st_data.oos_sources[out_port].has_vcc = true;
                } else if (in_port.second == GND_const) {
                    st_data.oos_gnd = in_port;
                    st_data.oos_sources[out_port].has_gnd = true;
                } else {
                    st_data.oos_sources[out_port].signal.push_back(in_port);
                }
            }
        }

        // parse "out_of_site_sinks"

        auto &out_of_site_sinks = json_data["out_of_site_sinks"];
        auto ous_sink_map = out_of_site_sinks.object_items();

        for (auto ous_sink_it = ous_sink_map.begin(); ous_sink_it != ous_sink_map.end(); ++ous_sink_it) {
            auto out_port = parseBelPinName(ous_sink_it->first);

            auto in_ports = ous_sink_it->second.array_items();
            for (auto const &in_port_it : in_ports) {
                auto in_port = parseBelPinName(in_port_it.string_value());
                st_data.oos_sinks[out_port].push_back(in_port);
            }
        }
    }
}

/**
 * @brief Builds a exclusive BEL set lookups.
 */
void NispHandler::identifyExclusiveBels() {

    // FIXME: Currently we disable binding two LUT BELs into a single LUT
    // element due to NISP shortcomings. Once they are resolved this won't
    // be necessary and the code below should be commented out / removed.

    const auto &tile_types = ctx->chip_info->tile_types;
    dict<IdString, pool<pool<IdString>>> bel_sets;

    // Loop over all LUT BELs
    for (const auto& tile_type : tile_types) {
        for (const auto& lut_element : tile_type.lut_elements) {

            IdString site_type;
            pool<IdString> bel_set;

            for (const auto& lut_bel : lut_element.lut_bels) {

                // Find the BEL
                int32_t ibel = -1;
                for (int32_t i=0; i<tile_type.bel_data.size(); ++i) {
                    const auto& bel_data = tile_type.bel_data[i];
                    if ((uint32_t)bel_data.name == lut_bel.name) {
                        ibel = i;
                        break;
                    }
                }
                NPNR_ASSERT(ibel != -1);

                // Determine site type
                const auto& bel_data = tile_type.bel_data[ibel];
                IdString st(tile_type.site_types[bel_data.site]);

                if (site_type == IdString()) {
                    site_type = st;
                } else {
                    NPNR_ASSERT(site_type == st);
                }

                // Insert
                bel_set.insert(IdString(lut_bel.name));
            }

            NPNR_ASSERT(site_type != IdString());
            bel_sets[site_type].insert(bel_set);
        }
    }

    // Build the lookup
    for (const auto& it : bel_sets) {
        const auto& site_type = it.first;
        const auto& bel_sets  = it.second;

        for (const auto& bel_set : bel_sets) {
            for (auto bel_name : bel_set) {

                pool<IdString> others;
                for (auto other : bel_set) {
                    others.insert(other);
                }

                exclusive_bels[site_type][bel_name] = others;
            }
        }
    }
}

/**
 * @brief Checks if the BELs of bound cells are in the same site
 *
 * @param first - pointer to the CellInfo structure of the first cell
 * @param second - pointer to the CellInfo structure of the second cell
 *
 * @return true if in the same site, false otherwise
 */
bool NispHandler::areCellsInTheSameSite(const CellInfo *first, const CellInfo *second) const
{
    NPNR_ASSERT(first != nullptr && this->isCellBound(first));
    NPNR_ASSERT(second != nullptr && this->isCellBound(second));

    if (first->bel.tile != second->bel.tile) {
        return false;
    }

    const BelInfoPOD &first_bel_pod = bel_info(ctx->chip_info, first->bel);
    const BelInfoPOD &second_bel_pod = bel_info(ctx->chip_info, second->bel);
    if (first_bel_pod.site != second_bel_pod.site) {
        return false;
    }

    return true;
}

/**
 * @brief Checks if a port is an input
 *
 * @param ref - pointer to the PortRef structure with information about the port
 *
 * @return true if port is input, false otherwise
 */
bool NispHandler::isPortInput(const PortRef *ref) const 
{
    NPNR_ASSERT(ref != nullptr);

    const PortInfo &user_port_info = ref->cell->ports.at(ref->port);
    if (user_port_info.type == PORT_IN || user_port_info.type == PORT_INOUT) {
        return true;
    }

    return false;
}

/**
 * @brief Checks if a port is an output
 *
 * @param ref - pointer to the PortRef structure with information about the port
 *
 * @return true if port is output, false otherwise
 */
bool NispHandler::isPortOutput(const PortRef *ref) const
{
    NPNR_ASSERT(ref != nullptr);

    const PortInfo &user_port_info = ref->cell->ports.at(ref->port);
    if (user_port_info.type == PORT_OUT || user_port_info.type == PORT_INOUT) {
        return true;
    }

    return false;
}

/**
 * @brief Checks if a cell is not bound to any BEL
 *
 * @param cell - pointer to the CellInfo structure of the given cell
 *
 * @return true if unbound, false if bound
 */
bool NispHandler::isCellBound(const CellInfo *cell) const
{
    NPNR_ASSERT(cell != nullptr);

    return (cell->bel != BelId()) && (this->ctx->getBoundBelCell(cell->bel) == cell);
}

/**
 * @brief Checks if all the "required" tags are not set for the site
 *        (requirements for the connection are met)
 *
 * @param[in] site_name - specific site name (e.g. SLICE_X1Y38.SLICEL)
 * @param[in] site_type_name - site name type (e.g. SLICEL)
 * @param[in] src - BEL.PIN name of the source connection (e.g. CLK.CLK)
 * @param[in] sink - BEL.PIN name of the sink connection (e.g. D5FF.CK)
 * @param[out] option - reference to a variable that will store the number of
 *                      the alternative connection used for the given source -> sink pair
 *
 * @return true if there exists one option that does not have any "required"
 *         tag set for the site (one of alternative connection options is possible),
 *         false otherwise
 */
bool NispHandler::checkRequiredTags(NetData *netd, IdString site,
    const RoutabilityInfo& routability, int *option, ConnStats *stats)
{
    NPNR_ASSERT(!routability.empty());

    const auto& storage = this->site_data.at(site);

    // site tag mask empty, all options are possible
    if (storage.tag_mask.size() == 0) {
        *option = 0;
        return true;
    }

    const BitVector& stored_tags = storage.tag_mask;
    BitVector& ignore_mask = this->getNetSiteData(netd, site).ignore_mask;

    // requirements available, check if there exists an option for which
    // none of the tags is set in the specified site
    for (size_t i = 0; i < routability.size(); ++i) {
        if (stats != nullptr)
            stats->checked_tag_sets_cnt++;

        // if there exist an option for which we haven't found any tags
        // it means that the requirements are met. Return true and save the
        // option number.
        const BitVector &req_tags = routability[i].requires;
        if (!(BitVector::andAny({&req_tags, &stored_tags, &ignore_mask}))) {
            *option = i;
            return true;
        }
    }

    // all alternatives use the required resources, connection not possible
    *option = -1;
    return false;
}

/**
 * @brief Perform an operation on a set of implied tags (active in site / deactivate in site)
 */
bool NispHandler::opImpliedTags(NetData *netd, ImpliedTagsOp op, IdString site,
    const RoutabilityInfo& routability, int option, ConnStats *stats)
{
    auto& storage = this->site_data.at(site);
    auto& imply = routability[option].implies;
    auto& ignore_mask = this->getNetSiteData(netd, site).ignore_mask;

    if (op == ImpliedTagsOp::ACTIVATE) {
        storage.tag_mask |= imply;
        ignore_mask &= ~imply;

        // TODO: Fix the iterator - it's broken and goes into an infinite loop
        //for (size_t tag_id : imply)
        //    storage.tag_cnt[tag_id]++;
        for (size_t tag_id = 0; tag_id < imply.size(); tag_id++) {
            if (imply.getBit(tag_id))
                storage.tag_cnt[tag_id]++;
        }

        if (stats)
            stats->activated_tag_sets_cnt++;
    }

    if (op == ImpliedTagsOp::DEACTIVATE) {
        // TODO: Fix the iterator - it's broken and goes into an infinite loop
        //for (size_t tag_id : imply) {
        //    storage.tag_cnt[tag_id]--;
        //    if (storage.tag_cnt[tag_id] == 0)
        //        storage.tag_mask.setBit(tag_id, false);
        //}
        for (size_t tag_id = 0; tag_id < imply.size(); tag_id++) {
            if (imply.getBit(tag_id)) {
                storage.tag_cnt[tag_id]--;

                if (storage.tag_cnt[tag_id] == 0) {
                    storage.tag_mask.setBit(tag_id, false);
                    ignore_mask.setBit(tag_id, true);
                }
            }
        }
    }

    return true;
}

#ifdef ENABLE_NISP_DUMPS
std::unordered_map<int/*option*/, std::vector<NispHandler::belpin_name_t>/*tags*/>
NispHandler::reportConflictingTags(IdString site_name, IdString site_type,
                                   const RoutabilityInfo& routability)
{
    std::unordered_map<int, std::vector<belpin_name_t>> option_tags;
    auto const &storage = this->site_data[site_name];

    // site tag mask empty, all options are possible
    if (storage.tag_mask.size() == 0) {
        return option_tags;
    }

    // check options
    for (size_t i = 0; i < routability.size(); ++i) {
        const auto& require = routability[i].requires;
        if (BitVector::andAny(require, storage.tag_mask)) {

            // Invert tag bit map to get tag names
            std::map<size_t, belpin_name_t> tag_map;
            for (const auto& it : site_tag_idx_map.at(site_type)) {
                tag_map[it.second] = it.first;
            }

            // Report required negative tags
            for (size_t j=0; j<require.size(); ++j) {
                if (require.getBit(j) && storage.tag_mask.getBit(j)) {
                    option_tags[i].push_back(tag_map.at(j));
                }
            }
        }
    }

    return option_tags;
}
#endif

const char* routability_str(NispHandler::Routability routability) {
    const char* status = "N/A";

    switch (routability) {
        case NispHandler::Routability::ROUTABLE:
            status = "routable";
            break;
        case NispHandler::Routability::UNROUTABLE:
            status = "unroutable";
            break;
        case NispHandler::Routability::NO_DATA:
            status = "unknown";
            break;
    }

    return status;
}

/**
 * @brief Updates the state of the NispHandler with information about the
 *        bound BEL and its connections.
 *
 * The function is responsible for updating NispHandler::site_data with
 * information about the added BELs, current site routability and
 * all introduced tags.
 *
 * @param bel - BEL to bind
 */
void NispHandler::bindBel(const BelId &bel)
{
    // get BEL name and its site name
    IdString site_name = get_site_name(this->ctx, bel);
    IdString bel_name = get_bel_name(this->ctx, bel);

    // check for BEL routability, created connections and implied tags
    //struct BelData bel_data {bel, false, 0};

    auto site_site_data_it = this->site_data.find(site_name);
    SiteData *site_site_data = nullptr;

    // If there is no information about the site or no entry for the given BEL create it
    if (site_site_data_it == this->site_data.end()) {
        site_site_data = &this->site_data[site_name];
        IdString site_type_name = get_site_type_name(ctx, bel);
        auto& st_data = this->site_type_data[site_type_name];
        site_site_data->tag_mask.resize(st_data.tag_cnt);
        site_site_data->tag_mask.clear();
        site_site_data->tag_cnt.resize(st_data.tag_cnt, 0);
    } else {
        site_site_data = &site_site_data_it->second;
    }

    CellInfo *cell = this->ctx->getBoundBelCell(bel);

    nisp_log("Binding cell %s to bel %s in site %s\n", cell->name.c_str(this->ctx),
             bel_name.c_str(this->ctx), site_name.c_str(this->ctx));

    bool success = true;

    // Check if any other BEL from exclusive BEL list is bound
    IdString st = get_site_type_name(ctx, bel);
    auto site_exclusive_bels_it = exclusive_bels.find(st);
    if (site_exclusive_bels_it != exclusive_bels.end()) {
        const auto& site_exclusive_bels = site_exclusive_bels_it->second;
        auto bel_set_it = site_exclusive_bels.find(bel_name);
        if (bel_set_it != site_exclusive_bels.end()) {
            const auto& bel_set = bel_set_it->second;

            for (auto& it : site_site_data->bound_cells) {
                auto* cell = it.first;
                auto& bind_info = it.second;

                if (isCellBound(cell)) {
                    IdString other_bel_name = get_bel_name(ctx, cell->bel);
                    if (bel_set.count(other_bel_name)) {
                        success = false;
                        break;
                    }
                }
            }
        }
    }

    // Build the pin map
    dict<IdString, std::vector<IdString>> pin_map;
    for (auto& ports_kv : cell->ports) {
        auto& port_name = ports_kv.first;
        auto& bel_pins  = ctx->getBelPinsForCellPin(cell, port_name);
        for (const auto& bel_pin : bel_pins) {
            pin_map[port_name].push_back(bel_pin);
        }
    }

    // Insert bound bound cell info here so that the pin map is in place
    // whenever something needs to be disconnected during connection attempts
    auto r = site_site_data->bound_cells.emplace(cell, CellBindInfo {success, pin_map, ConnStats()});
    NPNR_ASSERT(r.second);

    ConnStats stats;
    if (success) {
        for (auto& ports_kv : cell->ports) {
            auto& port_name = ports_kv.first;
            auto& port_info = ports_kv.second;

            if (port_info.net != nullptr)
                success &= this->tryToConnect(port_info.net, PortId(cell, port_name), &stats);
            if (!success)
                break;
        }
    }

    CellBindInfo* info = &r.first->second;
    info->success = success;
    info->last_bind_stats = stats;

    site_site_data->last_checked_cell = cell;

    if (!success)
        site_site_data->unroutable_bels++;

}

void implies_error(IdString site, std::pair<IdString, IdString> src_belpin,
                   std::pair<IdString, IdString> dst_belpin, int option, const BaseCtx *ctx)
{
    log_error("Could not imply %d tag set of connection %s.%s->%s.%s of site %s due to a tag "
              "conflict!\n", option,
              src_belpin.first.c_str(ctx), src_belpin.second.c_str(ctx),
              dst_belpin.first.c_str(ctx), dst_belpin.second.c_str(ctx),
              site.c_str(ctx));
}

/**
 * @brief Updates the state of the NispHandler with information about the
 *        unbound BEL and its connections.
 *
 * The function is responsible for updating NispHandler::site_data with
 * information about removed BELs, current site routability and all removed
 * tags.
 *
 * @param bel - BEL to unbind
 */
void NispHandler::unbindBel(const BelId &bel)
{
    IdString site_name = get_site_name(this->ctx, bel);
    auto& site_site_data = this->site_data[site_name];

    CellInfo *cell = this->ctx->getBoundBelCell(bel);
    NPNR_ASSERT(cell != nullptr);

    nisp_log("Unbinding cell %s from bel %s in site %s\n", cell->name.c_str(this->ctx),
             get_bel_name(this->ctx, bel).c_str(this->ctx), site_name.c_str(this->ctx));


    for (auto& ports_kv : cell->ports) {
        auto& port_name = ports_kv.first;
        auto& port_info = ports_kv.second;

        if (port_info.net != nullptr)
            this->disconnect(port_info.net, PortId(cell, port_name),
                             &site_site_data.bound_cells.at(cell).last_bind_stats);
    }

    if (!site_site_data.bound_cells.at(cell).success)
        site_site_data.unroutable_bels--;
    site_site_data.bound_cells.erase(cell);

#ifdef ENABLE_NISP_DUMPS
    // Remove all references to this BEL from failure data
    IdString bel_name = get_bel_name(ctx, bel);
    auto& failures = failure_data[site_name];
    for (auto it = failures.begin(); it != failures.end(); ) {
        if (it->src.site == site_name && it->src.belpin.first == bel_name) {
            it = failures.erase(it);
        }
        else if (it->dst.site == site_name && it->dst.belpin.first == bel_name) {
            it = failures.erase(it);
        }
        else {
            it++;
        }
    }
#endif
}

NispHandler::RoutabilityInfo* NispHandler::getRoutabilityInfo(SiteRoutabilityLookup_t st_lookup,
                                                              belpin_name_t src_belpin_name,
                                                              belpin_name_t dst_belpin_name)
{
    auto src_it = st_lookup.find(src_belpin_name);
    if (src_it == st_lookup.end())
        return nullptr;

    auto& src_lookup = src_it->second;

    auto dst_it = src_lookup.find(dst_belpin_name);
    if (dst_it == src_lookup.end())
        return nullptr;

    return &dst_it->second;
}

bool NispHandler::tryToConnect(NetInfo *net, const PortId port, ConnStats *stats)
{
    auto& netd = this->net_data[net];
    auto driver = PortId(net->driver.cell, net->driver.port);
    BelId port_bel = port.first->bel;
    IdString site = get_site_name(this->ctx, port_bel);

    nisp_log("Connecting port %s/%s to net %s\n", port.first->name.c_str(this->ctx),
             port.second.c_str(this->ctx), net->name.c_str(this->ctx));

    if (driver != port) {
        // We are placing a sink

        if (this->isCellBound(driver.first)) {
            IdString drv_site = get_site_name(this->ctx, driver.first->bel);

            // Try finding the simplest, direct intra-site connection
            if (drv_site == site) {
                if (this->tryToConnectIntraPorts(&netd, driver, port, stats))
                    return true;
            }

            //if (drv_site != site) {
            //    if (netd.intra_sink_site != site) {
            //        //NPNR_ASSERT(!netd.driver_valid);
            //        return false;
            //    }
            //}

            // Driver is in another site, or no direct connection was found to the sink.
            // We need to reach out-of-site.
            if (!this->tryToReachOutOfSiteSource(&netd, net, port, stats))
                return false;

            // Make sure the driver routes outside as well.
            IdString drivers_site = get_site_name(this->ctx, net->driver.cell->bel);
            if (this->placedOutOfSiteSinkCount(&netd, drivers_site) +
                netd.loopback_count == 0)
            {
                if (!this->tryToReachOutOfSiteSink(&netd, net, driver, stats)) {
                    // Rip-up - we can't be placed in a different site as the driver or use
                    // a loopback out-of-site connection.
                    this->disconnectOutOfSiteSource(&netd, net, port, stats);
                    return false;
                }
            }

            if (drivers_site == site)
                netd.loopback_count += 1;

            return true;

        } else {
            // Driver is unbound at the moment. In spirit of SiteRouter, let's reach
            // out-of-site source

            if (this->tryToReachOutOfSiteSource(&netd, net, port, stats))
                return true;

            return false;
        }
    } else {
        // We are placing a driver.

        bool routed_out_of_site = false;

        if (this->placedOutOfSiteSinkCount(&netd, site) != 0) {
            if (!this->tryToReachOutOfSiteSink(&netd, net, port, stats)) {
                // We are unable to reach out-of-site-sink
                netd.driver_valid = false;
                return false;
            }
            routed_out_of_site = true;
        }

        auto ns_data_it = netd.net_site_data.find(site);

        // Route sinks in driver's site
        if (ns_data_it == netd.net_site_data.end()) {
            netd.driver_valid = true;
            return true;
        }

        auto& ns_data = ns_data_it->second;

        bool sinks_routable = true;

        // TODO: Optimize this. Currently this data is copied because the iterator int he loop
        // below will get invalidated due to `placeSink` calls. Deferring th ecall to
        // `this->tryToReachOutOfSiteSink` might be enough if we assume that ading an exisitng
        // element to std::set won't invalidate the iterator (`this->tryToConnectIntraPorts`)
        std::set<PortId> sink_ports = ns_data.sink_ports;

        for (auto sink : sink_ports) {
            // Do not attempt to route to invalid sinks
            if (!this->site_data.at(site).bound_cells.at(sink.first).success)
                continue;

            // Try to "upgrade" the connection with sink to an intra-site connection
            this->disconnectOutOfSiteSource(&netd, net, sink, stats);
            if (this->tryToConnectIntraPorts(&netd, port, sink, stats)) {
                // XXX: This is wrong because there could've been more reasons for which the
                // binding was be invalid.
                //if (!this->site_data.at(site).bound_cells.at(sink.first).success) {
                //    this->site_data.at(site).bound_cells.at(sink.first).success = true;
                //    this->site_data.at(site).unroutable_bels--;
                //}
                continue;
            }
            // We couldn't make a direct connection with one of the sinks.
            // Revert to a loopback connection

            bool sink_available_for_loopback = /* false */ true;
            // TODO: Is this actually necessary?
            // !!! This could increase the number of users for no reason
            //if (this->tryToReachOutOfSiteSource(&netd, net, sink, stats)) {
            //    sink_available_for_loopback = true;
            //} else {
            //    // Sink's location might be actually valid, while driver's isn't
            //    nisp_log("WARNING: Driver is trying to route to an \"invalid\""
            //            "sink\n");
            //}

            // Assert that the sink is routed out-of-site
            //NPNR_ASSERT(netd.net_site_data.at(site).routing_options.at(sink).oos_pin >= 0);

            // Route the driver out-of site.
            if (!routed_out_of_site) {
                if (!this->tryToReachOutOfSiteSink(&netd, net, port, stats)) {
                    // Loopback is impossible - driver can't reach out-of-site
                    netd.driver_valid = false;
                    continue;
                    return false;
                }
                if (sink_available_for_loopback) {
                    if (!this->site_data.at(site).bound_cells.at(sink.first).success) {
                        this->site_data.at(site)
                            .bound_cells.at(sink.first).success = true;
                        this->site_data.at(site).unroutable_bels--;
                    }
                }
                netd.loopback_count++;
                routed_out_of_site = true;
            }
        }

        // Rip-up - we couldn't route ourselves to one of the sinks
        if (!sinks_routable) {
            for (auto sink : ns_data.sink_ports)
                this->disconnectIntraPorts(&netd, port, sink, stats);
            this->disconnectOutOfSiteSink(&netd, net, port, stats);

            if (ns_data.empty())
                netd.net_site_data.erase(site);

            netd.driver_valid = false;
            return false;
        }


        netd.driver_valid = true;
        return true;
    }
}

void NispHandler::disconnect(NetInfo *net, const PortId port, ConnStats *stats)
{
    nisp_log("Disconnecting port %s/%s from net %s\n", port.first->name.c_str(this->ctx),
             port.second.c_str(this->ctx), net->name.c_str(this->ctx));

    auto netd_it = this->net_data.find(net);
    if (netd_it == this->net_data.end()) {
        nisp_log("  wasn't connected\n");
        return;
    }
    auto& netd = netd_it->second;

    auto driver = PortId(net->driver.cell, net->driver.port);

    BelId port_bel = port.first->bel;
    IdString site = get_site_name(this->ctx, port.first->bel);

    if (driver != port) {
        // We are disconnecting a sink

        if (this->disconnectOutOfSiteSource(&netd, net, port, stats))
            nisp_log("  disconnected oos-source\n");

        bool driver_bound = this->isCellBound(driver.first);

        bool driver_needs_reroute = false;

        if (driver_bound) {
            IdString drv_site = get_site_name(this->ctx, driver.first->bel);
            if (site == drv_site) {
                if (this->disconnectIntraPorts(&netd, driver, port, stats))
                    nisp_log("  disconnected intra-ports\n");
                    if (!netd.driver_valid) {
                        // Try reconnecting a potentially invalid driver after removing a sink
                        driver_needs_reroute = true;
                    }
            }
        }

        auto ns_data_it = netd.net_site_data.find(site);

        if (ns_data_it != netd.net_site_data.end()) {
            ns_data_it->second.sink_ports.erase(port);
            if (ns_data_it->second.empty())
                netd.net_site_data.erase(site);
        }

        // Driver no longer requires to be routed out-of-site if there are no site-to-site
        // connections or loopbacks.
        // TODO: What if not all sinks are placed?
        if (driver_bound &&
            (this->placedOutOfSiteSinkCount(&netd, site) + netd.loopback_count == 0))
        {
            if (this->disconnectOutOfSiteSink(&netd, net, driver, stats))
                nisp_log("  disconnected driver's (%s/%s) oos-sink\n",
                         driver.first->name.c_str(this->ctx), driver.second.c_str(this->ctx));
        }

        // TODO: Fix this to avoid reconnecting to the sink we disconnect.
        //if (driver_needs_reroute)
        //    this->tryToConnect(net, PortId(net->driver.cell, net->driver.port), stats);
    } else {
        // We are disconnecting a driver

        if (this->disconnectOutOfSiteSink(&netd, net, port, stats))
            nisp_log("  disconnected oos-sink\n");
        netd.loopback_count = 0;

        // TODO: This is problematic, because there's currrently no mechanism to validate the
        // invalidated sinks back without removing and placing them again. This could be fixed
        // by binding/re-binding a BEL in `isBelLocationValid` instead of `bindBel`, but it could
        // also negatively impact performance.
        auto ns_data_it = netd.net_site_data.find(site);
        if (ns_data_it != netd.net_site_data.end()) {

            // TODO: Optimize this to avoid copying this data-structure.
            std::set<PortId> sink_ports = ns_data_it->second.sink_ports;
            for (auto sink : sink_ports) {
                if (this->disconnectIntraPorts(&netd, port, sink, stats)) {
                    nisp_log("  disconnected intra-ports\n");
                    if (!this->tryToReachOutOfSiteSource(&netd, net, sink, stats)) {
                        nisp_log("  failed to reconnect the sink to oos-source!\n");
                        auto sname = get_site_name(this->ctx, port_bel);
                        this->site_data[sname]
                            .bound_cells[sink.first].success = false;
                        this->site_data[sname].unroutable_bels++;
                    }
                }
            }
        }
    }
}

bool NispHandler::tryToConnectIntraPorts(NetData *netd, PortId driver, PortId sink,
                                         ConnStats *stats)
{
    IdString site = get_site_name(this->ctx, driver.first->bel);
    NPNR_ASSERT(site == get_site_name(this->ctx, sink.first->bel));
    IdString st = get_site_type_name(this->ctx, driver.first->bel);

    auto& st_data = this->site_type_data.at(st);

    auto& src_pins = ctx->getBelPinsForCellPin(driver.first, driver.second);
    auto& dst_pins = ctx->getBelPinsForCellPin(sink.first, sink.second);
    IdString src_bel = get_bel_name(this->ctx, driver.first->bel);
    IdString dst_bel = get_bel_name(this->ctx, sink.first->bel);

    nisp_log("driver %s/%s tries to reach sink %s/%s via intra-site routing\n",
             driver.first->name.c_str(this->ctx), driver.second.c_str(this->ctx),
             sink.first->name.c_str(this->ctx), sink.second.c_str(this->ctx));

    bool driver_already_placed = this->isCellBound(driver.first);
    bool driver_pin_locked = false;
    if (driver_already_placed) {
        for (auto p : src_pins)
            driver_pin_locked |= belpin_name_t(src_bel, p) == netd->driver_pin;
    }

    // TODO: Support usage of multiple driver pins. This would require changes in generic,
    // nextpnr code, see common/kernel/context.cc:47 (WireId Context::getNetinfoSourceWire)
    NPNR_ASSERT(src_pins.size() == 1);

    auto& ns_data = this->getNetSiteData(netd, site);
#ifdef ENABLE_NISP_DUMPS
    std::vector<FailureData> failures;
#endif

    for (IdString src_pin : src_pins) {
        auto src_belpin = belpin_name_t(src_bel, src_pin);

        if (driver_pin_locked) {
            // TODO: We should provide a mechanism which would allow choosing an alternative
            // driver if needed and attempting to swap routings. This is a sort of "backtracking".
            if (netd->driver_pin != src_belpin) {
                log_warning("Driver pin locked to something else that the source!\n");
                continue;
            }
        }

        auto from_it = st_data.p2p_routability.find(src_belpin);
        if (from_it == st_data.p2p_routability.end())
            continue;

        int dst_pin_idx;
        for (dst_pin_idx = 0; dst_pin_idx < (int)dst_pins.size(); dst_pin_idx++) {
            IdString dst_pin = dst_pins[dst_pin_idx];
            auto dst_belpin = belpin_name_t(dst_bel, dst_pin);

            auto to_it = from_it->second.find(dst_belpin);
            if (stats != nullptr)
                stats->conn_lookup_cnt++;
            if (to_it == from_it->second.end()) {
                break;
            }

            int option;
            if (this->checkRequiredTags(netd, site, to_it->second, &option, stats)) {
                if (!this->opImpliedTags(netd, ImpliedTagsOp::ACTIVATE, site, to_it->second, option,
                    stats))
                    implies_error(site, src_belpin, dst_belpin, option, this->ctx);
                ns_data.routing_options[dst_belpin] = {option, INTRA_SITE_OOS_PIN};
            } else {
#ifdef ENABLE_NISP_DUMPS
                FailureData data;
                data.type       = "intra";
                data.tags       = reportConflictingTags(site, st, to_it->second);
                data.src.site   = site;
                data.src.belpin = src_belpin;
                data.dst.site   = site;
                data.dst.belpin = dst_belpin;
                failures.push_back(data);
#endif
                break;
            }
        }

        if (dst_pin_idx == (int)dst_pins.size()) {
            netd->driver_pin = src_belpin;
            this->placeSink(netd, site, sink);
            nisp_log("  SUCCESS!\n");
            return true;
        }

        // We did not reach all destination pins, perform rip-up.
        for (--dst_pin_idx; dst_pin_idx >= 0; dst_pin_idx--) {
            IdString dst_pin = dst_pins[dst_pin_idx];
            auto dst_belpin = belpin_name_t(dst_bel, dst_pin);
            auto& routability = from_it->second.at(dst_belpin);
            auto rt_option = ns_data.routing_options.at(dst_belpin);
            this->opImpliedTags(netd, ImpliedTagsOp::DEACTIVATE, site, routability, rt_option.idx,
                                stats);
            ns_data.routing_options.erase(dst_belpin);
        }

    }

    if (ns_data.empty())
        netd->net_site_data.erase(site);

#ifdef ENABLE_NISP_DUMPS
    for (const auto& data : failures) {
        failure_data[site].push_back(data);
    }
#endif

    nisp_log("  FAILURE!\n");
    // No viable source was found
    return false;
}

bool NispHandler::disconnectIntraPorts(NetData *netd, PortId driver, PortId sink, ConnStats *stats)
{
    IdString site = get_site_name(this->ctx, driver.first->bel);
    NPNR_ASSERT(site == get_site_name(this->ctx, sink.first->bel));
    IdString st = get_site_type_name(this->ctx, driver.first->bel);

    auto& st_data = this->site_type_data.at(st);

    auto& s_data = this->site_data.at(site);
    const auto& bound_cell  = s_data.bound_cells.at(sink.first);
    const auto  dst_pins_it = bound_cell.pin_map.find(sink.second);
    if (dst_pins_it == bound_cell.pin_map.end()) {
        nisp_log("%s/%s seems unconnected\n",
            sink.first->name.c_str(ctx), sink.second.c_str(ctx));
        return true;
    }
    const auto& dst_pins = dst_pins_it->second;

    IdString src_bel = get_bel_name(this->ctx, driver.first->bel);
    IdString dst_bel = get_bel_name(this->ctx, sink.first->bel);

    if (!this->isCellBound(driver.first))
        return false;
    if (!this->isCellBound(sink.first))
        return false;

    if (netd->driver_pin.first != src_bel)
        return false;

    auto from_it = st_data.p2p_routability.find(netd->driver_pin);
    if (from_it == st_data.p2p_routability.end())
        return false;

    auto ns_data_it = netd->net_site_data.find(site);
    if (ns_data_it == netd->net_site_data.end())
        return false;

    auto& ns_data = ns_data_it->second;

    for (auto dst_pin : dst_pins) {
        auto dst_belpin = belpin_name_t(dst_bel, dst_pin);

        auto rt_option_it = ns_data.routing_options.find(dst_belpin);
        if (rt_option_it == ns_data.routing_options.end())
            return false;
        auto rt_option = rt_option_it->second;

        // TODO: Once we support mixed port-to-port connections (intra+loopback), this statement
        // should no longer return
        if (rt_option.oos_pin != INTRA_SITE_OOS_PIN)
            return false; // This was a loopback connection

        auto routability = from_it->second.at(dst_belpin);
        //if (routability_it == from_it->second.end())
        //    // Actually, there was no connection at all.
        //    // This is a rip-up from an incomplete attempt at creating one.
        //    return false;


        if (stats != nullptr)
            stats->conn_lookup_cnt++;


        // TODO: Support mixed connections (intra and loopback) between ports
        NPNR_ASSERT(rt_option.oos_pin == INTRA_SITE_OOS_PIN);

        this->opImpliedTags(netd, ImpliedTagsOp::DEACTIVATE, site, routability, rt_option.idx, stats);

        ns_data.routing_options.erase(rt_option_it);
    }

    if (ns_data.empty())
        netd->net_site_data.erase(site);

    this->removeSink(netd, site, sink);
    return true;
}

bool NispHandler::tryToReachOutOfSiteSource(NetData *netd, const NetInfo *net, PortId sink,
                                            ConnStats *stats)
{
    IdString site = get_site_name(this->ctx, sink.first->bel);
    IdString st = get_site_type_name(this->ctx, sink.first->bel);
    auto& st_data = this->site_type_data.at(st);
    auto& dst_pins = ctx->getBelPinsForCellPin(sink.first, sink.second);
    IdString dst_bel = get_bel_name(this->ctx, sink.first->bel);
    auto& port_site_data = this->site_data[site];

    nisp_log("sink %s/%s tries to reach oos-source\n",
             sink.first->name.c_str(this->ctx), sink.second.c_str(this->ctx));

    NPNR_ASSERT(this->isCellBound(sink.first));

    std::string dst_pins_str;
    for (size_t i = 0; i < dst_pins.size(); ++i) {
        IdString dst_pin = dst_pins[i];
        dst_pins_str += dst_pin.str(this->ctx) + " ";
    }
    nisp_log("BEL pins for %s/%s: %s\n",
             sink.first->name.c_str(this->ctx), sink.second.c_str(this->ctx),
             dst_pins_str.c_str());

    auto& ns_data = this->getNetSiteData(netd, site);

    using NetType = PhysicalNetlist::PhysNetlist::NetType;
    NetType net_type = this->ctx->get_net_type(net);

    int dst_pin_idx;
    for (dst_pin_idx = 0; dst_pin_idx < (int)dst_pins.size(); dst_pin_idx++) {
        IdString dst_pin = dst_pins[dst_pin_idx];
        auto dst_belpin = belpin_name_t(dst_bel, dst_pin);
        auto srcs_it = st_data.oos_sources.find(dst_belpin);
        if (srcs_it == st_data.oos_sources.end()) {
#ifdef ENABLE_NISP_DUMPS
            FailureData data;
            data.type       = "no-oos-source";
            data.src.site   = site;
            data.src.belpin = belpin_name_t();
            data.dst.site   = site;
            data.dst.belpin = dst_belpin;
            failure_data[site].push_back(data);
#endif
            break;
        }

#ifdef ENABLE_NISP_DUMPS
        std::vector<FailureData> failures;
#endif

        bool found_src = false;
        auto& srcs = srcs_it->second;

        // Try a dedicated constant source first, if the net is a constant net.
        int src_idx = 0;
        belpin_name_t dedicated_src;
        if ((net_type == NetType::VCC) && srcs.has_vcc) {
            dedicated_src = st_data.oos_vcc;
            src_idx = -1;
        } else if ((net_type == NetType::GND) && srcs.has_gnd) {
            dedicated_src = st_data.oos_gnd;
            src_idx = -1;
        }

        for (; src_idx < (int)srcs.signal.size(); src_idx++) {

            if (stats != nullptr)
                stats->oos_src_lookup_cnt++;

            auto src_belpin = (src_idx == -1) ? dedicated_src : srcs.signal[src_idx];

            if (!port_site_data.isOOSFree(net->name, src_belpin)) {
                // TODO: Disallow placement if a direct connection with another net is present
#ifdef ENABLE_NISP_DUMPS
                FailureData data;
                data.type       = std::string("oos-source-in-use (pin: ")
                                + src_belpin.first.c_str(this->ctx) + ", net: "
                                + port_site_data.getOOSUser(src_belpin).c_str(this->ctx) + ")";
                data.src.site   = site;
                data.src.belpin = src_belpin;
                data.dst.site   = site;
                data.dst.belpin = dst_belpin;
                failures.push_back(data);
#endif
                continue; // Pin is already in use
            }

            auto& routability = st_data.p2p_routability.at(src_belpin).at(dst_belpin);
            if (stats != nullptr)
                stats->conn_lookup_cnt++;

            int option;
            if (this->checkRequiredTags(netd, site, routability, &option, stats)) {
                if (!this->opImpliedTags(netd, ImpliedTagsOp::ACTIVATE, site, routability, option,
                    stats))
                    implies_error(site, src_belpin, dst_belpin, option, this->ctx);
                nisp_log("    activating pin (%s)\n", dst_belpin.second.c_str(this->ctx));
                ns_data.routing_options[dst_belpin] = RoutingOption {
                    option,
                    (src_idx == -1) ? ROUTING_DEDICATED_CONST_OOS_PIN : src_idx,
                };
                port_site_data.takeOOS(net->name, src_belpin);
                found_src = true;
                break;
            }
#ifdef ENABLE_NISP_DUMPS
            else {
                FailureData data;
                data.type       = "oos-source";
                data.tags       = reportConflictingTags(site, st, routability);
                data.src.site   = site;
                data.src.belpin = src_belpin;
                data.dst.site   = site;
                data.dst.belpin = dst_belpin;
                failures.push_back(data);
            }
#endif
        }

#ifdef ENABLE_NISP_DUMPS
        if (!found_src) {
            for (const auto& data : failures) {
                failure_data[site].push_back(data);
            }
        }
#endif

        if (!found_src)
            break;
    }


    if (dst_pin_idx == (int)dst_pins.size()) {
#ifdef ENABLE_NISP_DUMPS
        port_site_data.oos_src_conn_disconn_delta[sink]++;
#endif
        this->placeSink(netd, site, sink);
        nisp_log("  SUCCESS!\n");
        return true; // All pins got their sources
    }

    // We did not reach all destination pins, perform rip-up.
    for (--dst_pin_idx; dst_pin_idx >= 0; dst_pin_idx--) {
        IdString dst_pin = dst_pins[dst_pin_idx];
        auto dst_belpin = belpin_name_t(dst_bel, dst_pin);
        auto& rt_option = ns_data.routing_options.at(dst_belpin);
        NPNR_ASSERT(rt_option.oos_pin != INTRA_SITE_OOS_PIN);

        auto src_belpin = (rt_option.oos_pin == ROUTING_DEDICATED_CONST_OOS_PIN)
            ? ((net_type == NetType::VCC) ? st_data.oos_vcc : st_data.oos_gnd)
            : st_data.oos_sources.at(dst_belpin).signal[rt_option.oos_pin];

        auto& routability = st_data.p2p_routability.at(src_belpin).at(dst_belpin);

        this->opImpliedTags(netd, ImpliedTagsOp::DEACTIVATE, site, routability, rt_option.idx,
                            stats);
        ns_data.routing_options.erase(dst_belpin);
        port_site_data.returnOOS(net->name, src_belpin);
    }

    if (ns_data.empty())
        netd->net_site_data.erase(site);

#ifdef ENABLE_NISP_DUMPS
    auto &failures = failure_data.at(site);
    nisp_log("  FAILURE! (%s)\n", failures[failures.size() - 1].type.c_str());
#endif
    return false;
}

bool NispHandler::tryToReachOutOfSiteSink(NetData *netd, const NetInfo *net, PortId driver,
                                          ConnStats *stats)
{
    IdString site = get_site_name(this->ctx, driver.first->bel);
    IdString st = get_site_type_name(this->ctx, driver.first->bel);
    bool is_constants = st == this->ctx->id("$CONSTANTS");
    auto st_data = this->site_type_data.at(st);
    auto& src_pins = ctx->getBelPinsForCellPin(driver.first, driver.second);
    IdString src_bel = get_bel_name(this->ctx, driver.first->bel);
    auto& port_site_data = this->site_data[site];

    //bool driver_already_placed = this->isCellBound(driver.first);

    //bool driver_pin_locked = false;
    //if (driver_already_placed) {
    //    for (auto p : src_pins)
    //        driver_pin_locked |= belpin_name_t(src_bel, p) == netd->driver_pin;
    //}

    NPNR_ASSERT(src_pins.size() == 1);

    nisp_log("driver %s/%s tries to reach oos-sink\n",
             driver.first->name.c_str(this->ctx), driver.second.c_str(this->ctx));

    std::string src_pins_str;
    for (size_t i = 0; i < src_pins.size(); ++i) {
        IdString src_pin = src_pins[i];
        src_pins_str += src_pin.str(this->ctx) + " ";
    }
    nisp_log("BEL pins for %s/%s: %s\n",
             driver.first->name.c_str(this->ctx), driver.second.c_str(this->ctx),
             src_pins_str.c_str());

#ifdef ENABLE_NISP_DUMPS
    std::vector<FailureData> failures;
#endif /* ENABLE_NISP_DUMPS */

    for (IdString src_pin : src_pins) {
        auto src_belpin = belpin_name_t(src_bel, src_pin);

        // TODO: Support usage of multiple equivalent driver pins.
        //if (driver_pin_locked) {
        //    if (src_belpin != netd->driver_pin)
        //        continue;
        //}

        // XXX: "$CONSTANTS" virtual site is trivially routable to the outside.
        if (is_constants) {
            this->getNetSiteData(netd, site).routing_options[src_belpin] = {0, 0};
            netd->driver_pin = src_belpin;
            nisp_log("  SUCCESS!\n");
            return true;
        }

        auto sinks_it = st_data.oos_sinks.find(src_belpin);
        if (sinks_it == st_data.oos_sinks.end())
            continue;

        int sink_idx;
        for (sink_idx = 0; sink_idx < (int)sinks_it->second.size(); sink_idx++) {
            if (stats != nullptr)
                stats->oos_sink_lookup_cnt++;

            auto sink = sinks_it->second[sink_idx];

            if (!port_site_data.isOOSFree(net->name, sink)) {
#ifdef ENABLE_NISP_DUMPS
                FailureData data;
                data.type       = std::string("oos-sink-in-use: ")
                                + port_site_data.getOOSUser(sink).c_str(this->ctx);
                data.src.site   = site;
                data.src.belpin = src_belpin;
                data.dst.site   = site;
                data.dst.belpin = sink;
                failures.push_back(data);
#endif /* ENABLE_NISP_DUMPS */
                continue; // Pin is already in use
            }

            auto& routability = st_data.p2p_routability.at(src_belpin).at(sink);
            if (stats != nullptr)
                stats->conn_lookup_cnt++;

            int option;
            if (this->checkRequiredTags(netd, site, routability, &option, stats)) {
                if (!this->opImpliedTags(netd, ImpliedTagsOp::ACTIVATE, site, routability, option,
                    stats))
                    implies_error(site, src_belpin, sink, option, this->ctx);

                port_site_data.takeOOS(net->name, sink);

                // A little exception: For out-of-site sink. we store the driver pin instead of
                // sink
                this->getNetSiteData(netd, site).routing_options[src_belpin] = {option, sink_idx};
                netd->driver_pin = src_belpin;
                nisp_log("  SUCCESS!\n");
                return true;
            }
#ifdef ENABLE_NISP_DUMPS
            else {
                FailureData data;
                data.type       = "oos-sink";
                data.tags       = reportConflictingTags(site, st, routability);
                data.src.site   = site;
                data.src.belpin = src_belpin;
                data.dst.site   = site;
                data.dst.belpin = sink;
                failures.push_back(data);
            }
#endif
        }
    }

#ifdef ENABLE_NISP_DUMPS
    for (const auto& data : failures) {
        nisp_log("  FAILURE! (%s)\n", data.type.c_str());
        failure_data[site].push_back(data);
    }
#else
    nisp_log("  FAILURE!\n");
#endif
    return false;
}

bool NispHandler::disconnectOutOfSiteSource(NetData *netd, const NetInfo *net, PortId sink,
                                            ConnStats *stats)
{
    IdString site = get_site_name(this->ctx, sink.first->bel);
    IdString st = get_site_type_name(this->ctx, sink.first->bel);
    auto& st_data = this->site_type_data.at(st);
    IdString dst_bel = get_bel_name(this->ctx, sink.first->bel);

    auto& port_site_data = this->site_data[site];

    const auto& bound_cell = port_site_data.bound_cells.at(sink.first);
    const auto  dst_pins_it = bound_cell.pin_map.find(sink.second);
    if (dst_pins_it == bound_cell.pin_map.end()) {
        nisp_log("%s/%s seems unconnected\n",
            sink.first->name.c_str(ctx), sink.second.c_str(ctx));
        return true;
    }
    const auto& dst_pins = dst_pins_it->second;

    auto ns_data_it = netd->net_site_data.find(site);
    if (ns_data_it == netd->net_site_data.end())
        return false;
    auto& ns_data = ns_data_it->second;

    using NetType = PhysicalNetlist::PhysNetlist::NetType;
    NetType net_type = this->ctx->get_net_type(net);

    for (auto dst_pin : dst_pins) {
        auto dst_belpin = belpin_name_t(dst_bel, dst_pin);

        auto src_pins_it = st_data.oos_sources.find(dst_belpin);
        if (src_pins_it == st_data.oos_sources.end())
            return false;
        auto& src_pins = src_pins_it->second;

        auto rt_option_it = ns_data.routing_options.find(dst_belpin);
        if (rt_option_it == ns_data.routing_options.end())
            return false;

        auto rt_option = rt_option_it->second;

        if (rt_option.oos_pin == INTRA_SITE_OOS_PIN)
            return false; // This was intra-site routing.

        auto src_belpin = (rt_option.oos_pin == ROUTING_DEDICATED_CONST_OOS_PIN)
            ? ((net_type == NetType::VCC) ? st_data.oos_vcc : st_data.oos_gnd)
            : st_data.oos_sources.at(dst_belpin).signal[rt_option.oos_pin];
        auto& routability = st_data.p2p_routability.at(src_belpin).at(dst_belpin);
        this->opImpliedTags(netd, ImpliedTagsOp::DEACTIVATE, site, routability, rt_option.idx,
                            stats);
        ns_data.routing_options.erase(dst_belpin);
        port_site_data.returnOOS(net->name, src_belpin);
    }

#ifdef ENABLE_NISP_DUMPS
    port_site_data.oos_src_conn_disconn_delta[sink]--;
#endif

    this->removeSink(netd, site, sink);
    return true;
}

bool NispHandler::disconnectOutOfSiteSink(NetData *netd, const NetInfo *net, PortId driver,
                                          ConnStats *stats)
{
    IdString site = get_site_name(this->ctx, driver.first->bel);
    IdString st = get_site_type_name(this->ctx, driver.first->bel);

    auto ns_data_it = netd->net_site_data.find(site);
    if (ns_data_it == netd->net_site_data.end())
        return false;
    auto& ns_data = ns_data_it->second;

    // XXX: "$CONSTANTS" virtual site is trivially routable to the outside.
    if (st == ctx->id("$CONSTANTS")) {
        auto rt_option_it =  ns_data.routing_options.find(netd->driver_pin);
        if (rt_option_it == ns_data.routing_options.end())
            return false;
        ns_data.routing_options.erase(rt_option_it);
        if (ns_data.empty())
            netd->net_site_data.erase(site);
        return true;
    }

    auto& st_data = this->site_type_data.at(st);

    bool driver_already_placed = this->isCellBound(driver.first);

    if (!driver_already_placed)
        return false;

    //auto& src_pins = ctx->getBelPinsForCellPin(driver.first, driver.second);
    //IdString src_bel = get_bel_name(this->ctx, driver.first->bel);
    //
    //bool driver_pin_locked = false;
    //if (driver_already_placed) {
    //    for (auto p : src_pins)
    //        driver_pin_locked |= belpin_name_t(src_bel, p) == netd->driver_pin;
    //}
    //
    //if (!driver_pin_locked)
    //    return false;
    //NPNR_ASSERT(driver_pin_locked);

    auto sinks_it = st_data.oos_sinks.find(netd->driver_pin);

    if (sinks_it == st_data.oos_sinks.end())
        return false;

    auto& sinks = sinks_it->second;

    auto rt_option_it = ns_data.routing_options.find(netd->driver_pin);
    if (rt_option_it == ns_data.routing_options.end())
        return false;

    auto rt_option = rt_option_it->second;
    NPNR_ASSERT(rt_option.oos_pin != INTRA_SITE_OOS_PIN);
    auto sink = sinks[rt_option.oos_pin];
    auto& routability = st_data.p2p_routability.at(netd->driver_pin).at(sink);
    this->opImpliedTags(netd, ImpliedTagsOp::DEACTIVATE, site, routability, rt_option.idx, stats);
    this->site_data[site].returnOOS(net->name, sink);
    ns_data.routing_options.erase(netd->driver_pin);
    if (ns_data.empty())
        netd->net_site_data.erase(site);

    return true;
}

/**
 * @brief Check if the given BEL location in its site is valid
 *        from the NISP lookup perspective
 *
 * @param bel - BEL to check
 *
 * @return Routability::ROUTABLE if the BEL location causes the site to be routable,
 *         Routability::UNROUTABLE otherwise
 */
NispHandler::Routability NispHandler::isBelLocationValid(const BelId &bel)
{
    this->unbindBel(bel);
    this->bindBel(bel);

    site_name_t site = get_site_name(this->ctx, bel);
    SiteData &site_data = this->site_data.at(site);

    //return NispHandler::siteRoutability(site_data);
    return this->site_data.at(site).bound_cells.at(this->ctx->getBoundBelCell(bel)).success
        ? Routability::ROUTABLE
        : Routability::UNROUTABLE;
}

NispHandler::NetSiteData& NispHandler::getNetSiteData(NispHandler::NetData *netd, IdString site)
{
    return netd->net_site_data.emplace(site, this->site_data.at(site).tag_mask.size())
        .first->second;
}

const NispHandler::CellBindInfo& NispHandler::getCellBindInfo(const CellInfo *cell) const
{
    NPNR_ASSERT(this->isCellBound(cell));
    site_name_t site = get_site_name(this->ctx, cell->bel);
    return this->site_data.at(site).bound_cells.at(cell);
}


/* ========================================================================= *
 *                             NISP Site Dumping                             *
 * ========================================================================= */

enum class CellKind
{
    REGULAR,
    IN_PAD,
    OUT_PAD,
    IO_PAD,
};

std::string adjust_name_for_dump(std::string name) {
    std::string result = std::regex_replace(name, std::regex("\\$"), "\\$");
    result = std::regex_replace(result, std::regex("/"), "-");

    return name;
}

static CellKind cell_kind(const std::string& ctype)
{
    if (ctype == "$nextpnr_ibuf")
        return CellKind::IN_PAD;
    if (ctype == "$nextpnr_obuf")
        return CellKind::OUT_PAD;
    if (ctype == "$nextpnr_iobuf")
        return CellKind::IO_PAD;
    return CellKind::REGULAR;
}

static std::string extract_pure_site_name(const std::string &site_name)
{
    size_t dot_idx = site_name.find(".");
    if (dot_idx != std::string::npos)
        return site_name.substr(0, dot_idx);
    return site_name;
}

static std::string extract_pure_cell_name(const std::string &cell_name)
{
    std::string name;
    //TODO: Investigate what's the deal with "slash" cells.
    // Eg. `LUT6_2_6/LUT6` and `LUT6_2_6/LUT5`
    size_t slash_idx = cell_name.find("/");
    if (slash_idx != std::string::npos) {
        name = cell_name.substr(0, slash_idx) + "__"
            + cell_name.substr(slash_idx + 1, cell_name.size());
    } else {
        name = cell_name;
    }


    return adjust_name_for_dump(name);
}

std::string cell_pin_object(const CellInfo *cell, IdString port, Context *ctx) {
    std::string pure_cell_name = extract_pure_cell_name(cell->name.c_str(ctx));
    std::string type = cell->type.c_str(ctx);

    if (cell_kind(type) == CellKind::REGULAR) {
        return pure_cell_name + '/' + port.c_str(ctx);
    }

    return pure_cell_name;
}

void NispDumper::beginSite(const NispHandler& handler, IdString site_name)
{
    const NispHandler::SiteData &data = handler.site_data.at(site_name);
    IdString last_bel = get_bel_name(handler.ctx, data.last_checked_cell->bel);
    NispHandler::CellBindInfo last_bind = data.bound_cells.at(data.last_checked_cell);

    this->out
        << "========== SITE DUMP HEADER (" << site_name.c_str(handler.ctx) << ") =========\n"
        << "Dump index: " << this->dump_cnt << "\n"
        << "Total number of checks: " << handler.totalChecks << "\n"
        << "False-positives: " << handler.falsePositives << "\n"
        << "False-negatives: " << handler.falseNegatives << "\n"
        << "Site status: " << routability_str(handler.siteRoutability(data)) << "\n"
        << "Last checked Cell in this site: "
        << data.last_checked_cell->name.c_str(handler.ctx) << "\n"
        << "Last checked BEL in this site: "
        << last_bel.c_str(handler.ctx) << "\n"
        << "Number of routability lookups for last check: "
        << last_bind.last_bind_stats.conn_lookup_cnt << "\n"
        << "Number of alternatve OOS-source lookups for last check: "
        << last_bind.last_bind_stats.oos_src_lookup_cnt << "\n"
        << "Number of alternatve OOS-sink lookups for last check: "
        << last_bind.last_bind_stats.oos_sink_lookup_cnt << "\n"
        << "Number of checked alternative tag sets for last check: "
        << last_bind.last_bind_stats.checked_tag_sets_cnt << "\n"
        << "Number of activated alternative tag sets for last check: "
        << last_bind.last_bind_stats.activated_tag_sets_cnt << "\n"
        << "Unroutable BELs in site: " << data.unroutable_bels << "\n"
        << "Number of OOS sinks/sources used: " << data.used_oos.size() << "\n";

#ifdef ENABLE_NISP_DUMPS
    this->out << "Failed connections:\n";
    if (handler.failure_data.count(site_name)) {
        for (const auto& data : handler.failure_data.at(site_name)) {
            this->out << stringf(" %s.%s.%s -> %s.%s.%s (%s)\n",
                data.src.site.c_str(handler.ctx),
                data.src.belpin.first.c_str(handler.ctx),
                data.src.belpin.second.c_str(handler.ctx),
                data.dst.site.c_str(handler.ctx),
                data.dst.belpin.first.c_str(handler.ctx),
                data.dst.belpin.second.c_str(handler.ctx),
                data.type.c_str()
            );

            for (const auto& it : data.tags) {
                this->out << "  option " << it.first << " conflicts:\n";
                const auto& tags = it.second;
                for (const auto& tag : tags) {
                    this->out << "   " << tag.first.c_str(handler.ctx) << "."
                              << tag.second.c_str(handler.ctx) << "\n";
                }
            }
        }
    }
#endif

    this->out << "========== BEGIN SITE DUMP (" << site_name.c_str(handler.ctx)
              << ") ==========\n";
}

void NispDumper::endSite(const NispHandler& handler, IdString site_name)
{
    this->out << "========== END SITE DUMP (" << site_name.c_str(handler.ctx)
              << ") ============\n\n";

    this->flush();
    this->dump_cnt++;
}

#ifdef ENABLE_NISP_DUMPS
void NispHandler::dumpSiteTcl(IdString site, BelId bel, IdString cell_name, 
                              bool include_all_net_users, bool headers_only)
{
    nisp_log("Dumping NISP state...\n");

    using namespace vivado;

    NispDumper& out = *this->ctx->nisp_dumper;

    SiteData &data = this->site_data.at(site);

    out.beginSite(*this, site);

    if (headers_only) {
        out << comment("`headers_only` option is set to true. Site dump skipped");
        out.endSite(*this, site);
        return;
    }

    std::unordered_set<const NetInfo *> nets;
    pool<IdString> cells;


    out << comment("Delta of oos-source connections/disconnections:");
    for (auto &kv : data.oos_src_conn_disconn_delta) {
        auto port = kv.first;
        out << comment("  " + extract_pure_cell_name(port.first->name.c_str(this->ctx)) +
                       "." + port.second.c_str(this->ctx) + ": " +
                       std::to_string(kv.second));
    }

    for (auto& kv : data.bound_cells) {
        const CellInfo *cell = kv.first;

        for (auto& ports_kv : cell->ports) {
            auto& port_info = ports_kv.second;

            auto* net = port_info.net;
            if (net != nullptr) {
                auto add_pin = [&](PortRef& user) {
                    if ((user.cell != cell) && !include_all_net_users)
                        return;

                    if (cells.find(user.cell->name) == cells.end()) {
                        out << DeclareCell(user.cell);

                        if (this->isCellBound(user.cell)) {
                            IdString st = get_site_type_name(this->ctx, user.cell->bel);

                            // TODO: Fix this check. For some reason it passes when it shouldn't
                            if (st != this->ctx->id("$CONSTANTS"))
                                out << allow_error(PlaceCell(user.cell, user.cell->bel));
                        }

                        cells.insert(user.cell->name);
                    }


                    nets.emplace(net);
                };
                for (auto& user : net->users)
                    add_pin(user);
                if (net->driver.cell != nullptr)
                    add_pin(net->driver);
            }
            else {
                out << comment(stringf("%s.%s is unconnected",
                    cell->name.c_str(ctx),
                    port_info.name.c_str(ctx)
                ));
            }
        }
    }

    this->highlightNispStateTcl(out, site);

    // Mark the site and BEL
    out << highlight(Site(site), Color::YELLOW, HighlightKind::MARK)
        << highlight(Bel(bel), Color::YELLOW, HighlightKind::MARK);

    for (auto net : nets) {
        // TODO: Make ConnectToNet generic for various containers
        std::vector<PortRef> users;
        for (auto user : net->users)
            users.push_back(user);

        out << CreateNet(net->name)
            << ConnectToNet(net->name, users);
    }

    out.endSite(*this, site);
}

void NispHandler::highlightNispStateTcl(vivado::TclWriter& out, IdString site) const {
    using namespace vivado;

    auto site_data_it = this->site_data.find(site);
    if (site_data_it == this->site_data.end())
        return;
    auto& site_data = site_data_it->second;
    auto failures_it = this->failure_data.find(site);

    if (site_data.bound_cells.size() == 0) {
        log_warning("NISP site data without bound cells: %s\n", site.c_str(this->ctx));
        return;
    }

    IdString site_type = get_site_type_name(ctx, site_data.bound_cells.begin()->first->bel);
    if (site_type == this->ctx->id("$CONSTANTS"))
        return;

    // Highlight failed Cells/Bels
    for (auto& binding : site_data.bound_cells) {
        auto* cell = binding.first;
        auto& cell_binding = binding.second;

        Color hl_color = cell_binding.success ? Color::GREEN : Color::RED;

        out << highlight(Bel(cell->bel), hl_color);
    }

    // Invert tag name map to get tags names
    std::unordered_map<size_t, belpin_name_t> tag_map;
    for (const auto& it : this->site_tag_idx_map.at(site_type)) {
        tag_map[it.second] = it.first;
    }

    // Highlight active tags
    for (size_t i = 0; i < site_data.tag_cnt.size(); ++i) {
        if (site_data.tag_cnt[i] == 0) {
            continue;
        }

        belpin_name_t tag = tag_map.at(i);
        IdString bel_name = tag.first;
        IdString pin_name = tag.second;

        out << comment("Tag count: " + std::to_string((site_data.tag_cnt[i])))
            << highlight(NispDumper::BelPinName(site, bel_name, pin_name),
                         Color::YELLOW);
    }

    // Highlight active OOS sources/sinks
    for (const auto& it : site_data.used_oos) {
        IdString bel_name = it.first.first;
        IdString pin_name = it.first.second;

        const std::string color = "blue";

        out << comment(std::string("OOS for net ") + it.second.net.c_str(this->ctx) +
                       " (users: " + std::to_string(it.second.users) + ")")
            << highlight(NispDumper::BelPinName(site, bel_name, pin_name),
                         Color::BLUE);
    }

    // Highlight failed pins
    if (failures_it == this->failure_data.end()) {
        out << comment(std::string("<failure data missing for site ") +
                       site.c_str(this->ctx) + ">");
    } else {
        for (const auto& data : failures_it->second) {
            out << comment("FAILURE: " + data.type)
                << comment(std::string("  source: ") + data.src.site.c_str(this->ctx) + "/" +
                        data.src.belpin.first.c_str(this->ctx) + "." +
                        data.src.belpin.second.c_str(this->ctx))
                << comment(std::string("  destination: ") + data.dst.site.c_str(this->ctx) + "/" +
                        data.dst.belpin.first.c_str(this->ctx) + "." +
                        data.dst.belpin.second.c_str(this->ctx))
                << highlight(NispDumper::BelPinName(
                        data.src.site,
                        data.src.belpin.first,
                        data.src.belpin.second),
                    Color::MAGENTA
                );
        }
    }
}

void NispHandler::highlightNispStateTcl(vivado::TclWriter& out) const {
    for (auto& site_kv : this->site_data)
        this->highlightNispStateTcl(out, site_kv.first);
}

#endif /* ENABLE_NISP_DUMPS */

void NispDumper::BelPinName::tclGetRef(std::ostream& out, const Context *ctx) const
{
    // TODO: Make a proper detection of site-pin instead of this workaround
    bool is_site_pin =
        (this->bel_name == this->pin_name) && (this->bel_name.str(ctx) != "PAD");

    if (is_site_pin) {
        std::string pure_site_name = extract_pure_site_name(this->site_name.c_str(ctx));
        out << "get_site_pins -of_objects [get_sites " << pure_site_name
            << "] " << pure_site_name << "/"
            << this->pin_name.c_str(ctx);
    } else {
        out << "get_bel_pins "<< extract_pure_site_name(this->site_name.c_str(ctx)) << "/"
            << this->bel_name.c_str(ctx) << "/" << this->pin_name.c_str(ctx);
    }
}

/* ========================================================================= *
 *                           Other functions                                 *
 * ========================================================================= */

/**
 * @brief Get site type name for the given BEL
 *
 * @param ctx - pointer to the main Context class
 * @param bel - reference to the BelId with information about the BEL
 *
 * @return site type name of the BEL
 */
IdString get_site_type_name(const Context *ctx, const BelId &bel)
{
    const BelInfoPOD &bel_pod = bel_info(ctx->chip_info, bel);

    const auto &tile_types = ctx->chip_info->tile_types;
    const auto &tiles = ctx->chip_info->tiles;
    const auto &tile = tiles[bel.tile];
    const auto &tile_type = tile_types[tile.type];

    return IdString(tile_type.site_types[bel_pod.site]);
}

/**
 * @brief Get site name for the given BEL
 *
 * @param ctx - pointer to the main Context class
 * @param bel - reference to the BelId with information about the BEL
 *
 * @return site name of the BEL
 */
IdString get_site_name(const Context *ctx, const BelId &bel)
{
    const SiteInstInfoPOD &site_inst = ctx->get_site_inst(bel);
    return ctx->id(site_inst.name.get()); // FIXME: This would be slow
}

 /**
 * @brief Get BEL name for the given BEL
 *
 * @param ctx - pointer to the main Context class
 * @param bel - reference to the BelId with information about the BEL
 *
 * @return BEL name
 */
IdString get_bel_name(Context *ctx, const BelId &bel)
{
    const BelInfoPOD &bel_pod = bel_info(ctx->chip_info, bel);
    return IdString(bel_pod.name);
}

 /**
 * @brief Get BEL pin names for the given BEL
 *
 * @param ctx - pointer to the main Context class
 * @param bel - reference to the BelId with information about the BEL
 *
 * @return BEL pin name vector
 */

const std::vector<IdString>& get_bel_pin_names_for_port(const Context *ctx, const PortRef *ref)
{
    return ctx->getBelPinsForCellPin(ref->cell, ref->port);
}

} /* namespace nisp */
NEXTPNR_NAMESPACE_END
