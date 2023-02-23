#ifndef NISP_H
#define NISP_H

#include <forward_list>
#include <set>
#include <map>
#include <tuple>

#include "chipdb.h"
#include "nextpnr_types.h"
#include "bvec.h"
#include "vivado.h"

//#define NISP_VERBOSE

#ifdef NISP_VERBOSE
    #define nisp_log(...) log_info( __VA_ARGS__ )
    #define nisp_log_color(...) log_info_color( __VA_ARGS__ )
#else
    #define nisp_log(...) (void)(NULL)
    #define nisp_log_color(...) (void)(NULL)
#endif

NEXTPNR_NAMESPACE_BEGIN

class Context;
namespace nisp {

struct NispLookupValueNotFound : public std::exception
{
    const char *what() { return "Routability value not found in lookup table"; }
};

using bel_idx_t = int32_t;
using constid_t = int32_t;
using flat_pin_idx_t = int32_t;
using pin_idx_t = int32_t;
using site_idx_t = int32_t;
using site_type_idx_t = int32_t;
using site_variant_t = int16_t;
using tile_idx_t = int32_t;
using tile_type_idx_t = int32_t;

class NispDumper;

/**
 * @brief Class used to handle all functionalities related with NISP
 */
class NispHandler
{

    /* type definitions */

  public:
    enum class Routability { ROUTABLE, UNROUTABLE, NO_DATA };

    struct ConnStats {
        ConnStats() = default;

        size_t oos_src_lookup_cnt = 0;
        size_t oos_sink_lookup_cnt = 0;
        size_t conn_lookup_cnt = 0;
        size_t checked_tag_sets_cnt = 0;
        size_t activated_tag_sets_cnt = 0;
    };

    struct CellBindInfo {
        bool success = false;
        dict<IdString/*Cell pin*/, std::vector<IdString>/*BEL pin(s)*/> pin_map;
        ConnStats last_bind_stats;
    };

  private:
    using site_name_t = IdString;
    using site_type_name_t = IdString;
    using belpin_name_t = std::pair<IdString, IdString>;
    using tag_count_t = int;
    using PortId = std::pair<CellInfo *, IdString>; /* cell, port */

    enum class ImpliedTagsOp {
        ACTIVATE,
        DEACTIVATE,
    };

    struct PinPairRoutability {
        PinPairRoutability() = default;
        PinPairRoutability(size_t tag_cnt) : implies(tag_cnt), requires(tag_cnt) {}

        BitVector implies;
        BitVector requires;
    };

    using RoutabilityInfo = std::vector<PinPairRoutability>;

    struct BelData
    {
        BelId bel;
        bool loc_is_valid;
        int users = 0;
        int last_conn_check_count = 0;
    };

    struct UsedOOS {
        IdString net;
        size_t users = 0;
    };

    struct SiteData
    {
        //std::map<tag_name_t, tag_count_t> tags;
        BitVector tag_mask;
        std::vector<size_t> tag_cnt;
        //std::map<int32_t, BelData> bel_data;
        std::map<const CellInfo *, CellBindInfo> bound_cells;
        CellInfo *last_checked_cell = nullptr;
        int unroutable_bels = 0;

#ifdef ENABLE_NISP_DUMPS
        std::map<PortId, int> oos_src_conn_disconn_delta;
#endif /* ENABLE_NISP_DUMPS */

    protected:
        // OOS sinks/sources used, mapped to users's (net) name
        std::map<belpin_name_t, UsedOOS> used_oos;

        friend class NispHandler;
        friend class NispDumper;

    public:

        IdString getOOSUser(belpin_name_t oos_pin) {
            return this->used_oos.at(oos_pin).net;
        }

        bool isOOSFree(IdString net, belpin_name_t oos_pin) {
            auto used_it = this->used_oos.find(oos_pin);
            if ((used_it != this->used_oos.end()) && (used_it->second.net != net)) {
                return false;
            }
            return true;
        }

        void takeOOS(IdString net, belpin_name_t oos_pin) {
            auto r = this->used_oos.emplace(oos_pin, UsedOOS {net, 1});
            if (!r.second) {
                NPNR_ASSERT(r.first->second.net == net);
                r.first->second.users++;
            }
        }

        void returnOOS(IdString net, belpin_name_t oos_pin) {
            auto& used_oos = this->used_oos.at(oos_pin);
            NPNR_ASSERT(used_oos.net == net);
            used_oos.users--;
            if (used_oos.users == 0)
                this->used_oos.erase(oos_pin);
        }
    };

    #define INTRA_SITE_OOS_PIN (-1)
    #define ROUTING_DEDICATED_CONST_OOS_PIN (-2)

    struct RoutingOption {
        // Option index
        int idx = -1;
        // Used when the driver is out-of-site. Index of the source or sink pin.
        // If the routing is intra-site it's set to INTRA_SITE_OOS_PIN.
        // If the routing ROUTING_DEDICATED_CONST_OOS_PIN.
        int oos_pin = -1;
    };

    struct NetData;

    struct NetSiteData {
        NetSiteData(size_t tag_count) {
            this->ignore_mask.resize(tag_count);
            this->ignore_mask.clear();
        }
        // Maps pins to routing options used for routes from the driver. In case the driver
        // is out-of-site the opotion desribes the routing from an out-of-site sink.
        std::map<belpin_name_t, RoutingOption> routing_options;
        // Caches sinks located in this site
        std::set<PortId> sink_ports;
        // Tags in this site implied by this net. These whould be ignored when checking tags.
        BitVector ignore_mask;

        bool empty() const {
            return this->routing_options.empty() && this->sink_ports.empty();
        }
    };

    struct NetData {
        // BEL pin name of the driver (site can be retrieved by checking driving port of a net)
        // This gets updated only for valid placements.
        belpin_name_t driver_pin;
        // Was the driver of the net bound validly.
        // If invalid, routing should be retried after moving and intra-site sink
        bool driver_valid = false;
        // Total number of placed sinks
        size_t placed_sinks = 0;
        // Number of loopback connections
        size_t loopback_count = 0;
        // Per-site data associated with this net.
        std::map<IdString, NetSiteData> net_site_data;
    };

    struct OOSSourceList {
        std::vector<belpin_name_t> signal;
        bool has_vcc = false;
        bool has_gnd = false;
    };

    using OOSSinkList = std::vector<belpin_name_t>;

    struct SiteTypeData {
        // Routability between pairs of BEL pins
        dict<belpin_name_t, dict<belpin_name_t, RoutabilityInfo>> p2p_routability;
        // Out-of-site sources for given sinks
        dict<belpin_name_t, OOSSourceList> oos_sources;
        // Out-of-site sinks for given drivers
        dict<belpin_name_t, OOSSinkList> oos_sinks;
        // Index of nextpnr's dedicated VCC source. -1 if not present;
        belpin_name_t oos_vcc;
        // Index of nextpnr's dedicated GND source. -1 if not present;
        belpin_name_t oos_gnd;
        // Lenghth of tag mask
        size_t tag_cnt = 0;
    };

#ifdef ENABLE_NISP_DUMPS
    struct FailureData {
        // Type
        std::string type;
        // Conflicting tags
        std::unordered_map<int, std::vector<belpin_name_t>> tags;

        // Source endpoint
        struct {
            site_name_t   site;
            belpin_name_t belpin;
        } src;

        // Destination endpoint
        struct {
            site_name_t   site;
            belpin_name_t belpin;
        } dst;
    };
#endif

    using SiteRoutabilityLookup_t =
            dict<belpin_name_t, dict<belpin_name_t, RoutabilityInfo>>;

    /* class members */

  public:
    NispHandler(Context *ctx) : ctx(ctx){};
    virtual ~NispHandler() = default;

    Routability isBelLocationValid(const BelId &bel);

    void fillLookup(std::string);
    void identifyExclusiveBels();
    void bindBel(const BelId &bel);
    void unbindBel(const BelId &bel);

#ifdef ENABLE_NISP_DUMPS
    void dumpSiteTcl(IdString site, BelId bel, IdString cell_name, bool include_all_net_users = false,
                     bool headers_only = false);
    void highlightNispStateTcl(vivado::TclWriter& out, IdString site) const;
    void highlightNispStateTcl(vivado::TclWriter& out) const;
#endif /* ENABLE_NISP_DUMPS */

    const CellBindInfo& getCellBindInfo(const CellInfo *cell) const;

    size_t totalChecks = 0;
    size_t falsePositives = 0;
    size_t falseNegatives = 0;
    size_t misses = 0;

  private:
    Context *ctx;

    std::map<site_type_name_t, SiteTypeData> site_type_data;
    std::map<site_name_t, SiteData> site_data;
    std::map<NetInfo *, NetData> net_data;

#ifdef ENABLE_NISP_DUMPS
    std::map<site_name_t, dict<belpin_name_t, size_t>> site_tag_idx_map;
    std::map<IdString, std::vector<FailureData>> failure_data;
#endif

    dict<IdString/*site type*/,dict<IdString/*BEL name*/, pool<IdString/*BEL name*/>>> exclusive_bels;

    bool areCellsInTheSameSite(const CellInfo *src_cell, const CellInfo *sink_cell) const;
    bool isPortInput(const PortRef *port_ref) const;
    bool isPortOutput(const PortRef *port_ref) const;
    bool isCellBound(const CellInfo *cell) const;

    // Attempts to connect a port to a net.
    // Return true on success, false on failure.
    bool tryToConnect(NetInfo *net, const PortId port, ConnStats *stats);
    // Disconnects port from a net.
    void disconnect(NetInfo *net, const PortId port, ConnStats *stats);

    // Attempts to connect two ports via a direct route within a single site.
    // Returns `true` on success
    // TODO: This interface does not support port-to-port connections which mix intra-site
    // connections with loopback connections. It should be redesigned to take that into account.
    bool tryToConnectIntraPorts(NetData *netd, PortId driver, PortId sink, ConnStats *stats);
    // Removes a direct connection between two ports within a site.
    // Returns `true` if the connection was present, otherwise `false`.
    bool disconnectIntraPorts(NetData *netd, PortId driver, PortId sink, ConnStats *stats);
    // Attempts to expose a sink to global routing by creating a route to an out-of-site source.
    // Returns true on success.
    bool tryToReachOutOfSiteSource(NetData *netd, const NetInfo *net, PortId sink,
                                   ConnStats *stats);
    // Attempts to expose a driver to global routing by creating a route to an out-of-site sink.
    // Returns true on success.
    bool tryToReachOutOfSiteSink(NetData *netd, const NetInfo *net, PortId driver,
                                 ConnStats *stats);
    // Disconnects a sink from global routing.
    // Returns `true` if the sink was connected, otherwise `false`.
    bool disconnectOutOfSiteSource(NetData *netd, const NetInfo *net, PortId sink,
                                   ConnStats *stats);
    // Disconnects a driver from global routing.
    // Returns `true` if the driver was connected, otherwise `false`.
    bool disconnectOutOfSiteSink(NetData *netd, const NetInfo *net, PortId driver,
                                 ConnStats *stats);

    bool checkRequiredTags(NetData *net, IdString site_name, const RoutabilityInfo& routability,
                           int *option, ConnStats *stats);

    bool opImpliedTags(NetData *netd, ImpliedTagsOp op, IdString site_name,
                       const RoutabilityInfo& routability, int option, ConnStats *stats);

#ifdef ENABLE_NISP_DUMPS
    std::unordered_map<int/*option*/, std::vector<belpin_name_t>/*tags*/> reportConflictingTags (
        IdString site_name, IdString site_type, const RoutabilityInfo& routability);
#endif

    RoutabilityInfo* getRoutabilityInfo(SiteRoutabilityLookup_t st_lookup,
                                        belpin_name_t src_belpin_name,
                                        belpin_name_t dst_belpin_name);

    bool calculateBelRoutability(struct BelData &bel_data);

    NetSiteData& getNetSiteData(NetData *netd, IdString site);

    void placeSink(NetData *netd, IdString site, PortId port) {
        auto& ns_data = this->getNetSiteData(netd, site);
        ns_data.sink_ports.insert(port);
        netd->placed_sinks++;
    }

    void removeSink(NetData *netd, IdString site, PortId port) {
        auto& ns_data = netd->net_site_data.at(site);
        size_t r = ns_data.sink_ports.erase(port);
        NPNR_ASSERT(r != 0);
        if (ns_data.empty())
            netd->net_site_data.erase(site);
        netd->placed_sinks--;
    }

    size_t placedOutOfSiteSinkCount(NetData *netd, IdString site) const {
        auto ns_data_it = netd->net_site_data.find(site);
        if (ns_data_it == netd->net_site_data.end())
            return netd->placed_sinks;

        // If we forgot to perform some cleanup, this will fail
        NPNR_ASSERT(!ns_data_it->second.empty());

        return netd->placed_sinks - ns_data_it->second.sink_ports.size();
    }

    static NispHandler::Routability siteRoutability(const class NispHandler::SiteData& site_data)
    {
        if (site_data.unroutable_bels == 0) {
            return Routability::ROUTABLE;
        }
        return Routability::UNROUTABLE;
    }

    friend class NispDumper;
};

IdString get_site_type_name(const Context *ctx, const BelId &bel);
IdString get_site_name(const Context *ctx, const BelId &bel);

IdString get_bel_name(Context *ctx, const BelId &bel);
const std::vector<IdString>& get_bel_pin_names_for_port(const Context *ctx, const PortRef *ref);

#define NISP_DUMP_PATH "nisp.dump"

class NispDumper : public vivado::TclWriter {
  public:
    void beginSite(const NispHandler& handler, IdString site_name);
    void endSite(const NispHandler& handler, IdString site_name);

    struct BelPinName : public vivado::DesignResource {
        BelPinName(IdString site_name_, IdString bel_name_, IdString pin_name_)
            : site_name(site_name_)
            , bel_name(bel_name_)
            , pin_name(pin_name_)
        {}
        void tclGetRef(std::ostream& out, const Context *ctx) const override;
        IdString site_name;
        IdString bel_name;
        IdString pin_name;
    };

    NispDumper(const Context *ctx)
        : vivado::TclWriter(NISP_DUMP_PATH, ctx)
        , dump_cnt(0) {}

  private:

    int dump_cnt = 0;
};

} // namespace nisp
NEXTPNR_NAMESPACE_END
#endif /* NISP */
