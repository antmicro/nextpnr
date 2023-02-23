#include "vivado.h"
#include "vivado_impl.h"

#include <regex>

#include "context.h"

NEXTPNR_NAMESPACE_BEGIN

using namespace vivado;

enum class CellKind {
    REGULAR,
    IN_PAD,
    OUT_PAD,
    IO_PAD,
};

std::string adjust_name_for_tcl(const std::string& name) {
    std::string result = std::regex_replace(name, std::regex("\\$"), "\\$");
    result = std::regex_replace(result, std::regex("/"), "-");

    return result;
}

static CellKind cell_kind(const std::string& ctype) {
    if (ctype == "$nextpnr_ibuf")
        return CellKind::IN_PAD;
    if (ctype == "$nextpnr_obuf")
        return CellKind::OUT_PAD;
    if (ctype == "$nextpnr_iobuf")
        return CellKind::IO_PAD;
    return CellKind::REGULAR;
}

static std::string extract_pure_site_name(const std::string &site_name) {
    size_t dot_idx = site_name.find(".");
    if (dot_idx != std::string::npos)
        return site_name.substr(0, dot_idx);
    return site_name;
}

static std::string extract_pure_cell_name(const std::string &cell_name) {
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


    return adjust_name_for_tcl(name);
}

IdString get_bel_name(const Context *ctx, const BelId &bel) {
    const BelInfoPOD &bel_pod = bel_info(ctx->chip_info, bel);
    return IdString(bel_pod.name);
}

IdString get_site_name(const Context *ctx, const BelId &bel)
{
    const SiteInstInfoPOD &site_inst = ctx->get_site_inst(bel);
    return ctx->id(site_inst.name.get()); // FIXME: This would be slow
}

std::string cell_pin_name(PortRef port_ref, const Context *ctx) {
    std::string pure_cell_name = extract_pure_cell_name(port_ref.cell->name.c_str(ctx));
    std::string type = port_ref.cell->type.c_str(ctx);

    if (cell_kind(type) == CellKind::REGULAR) {
        return pure_cell_name + '/' + port_ref.port.c_str(ctx);
    }

    return pure_cell_name;
}

const char *vivado::vivado_highlight_color_map[] = {
    /* [Color::RED] = */     "red",
    /* [Color::GREEN] = */   "green",
    /* [Color::BLUE] = */    "blue",
    /* [Color::YELLOW] = */  "yellow",
    /* [Color::MAGENTA] = */ "magenta",
};

// -----------------------------------------------------------

void Net::tclGetRef(std::ostream& out, const Context *ctx) const {
    out << "get_nets " << this->name.c_str(ctx);
}

void Site::tclGetRef(std::ostream& out, const Context *ctx) const {
    out << "get_sites " << extract_pure_site_name(this->name.c_str(ctx));
}

void Cell::tclGetRef(std::ostream& out, const Context *ctx) const {
    out << "get_cells " << this->cell->name.c_str(ctx);
}

void Bel::tclGetRef(std::ostream& out, const Context *ctx) const {
    IdString site = get_site_name(ctx, bel);
    IdString bel_name = get_bel_name(ctx, bel);
    out << "get_bels " << extract_pure_site_name(site.str(ctx)) << "/"
        << bel_name.c_str(ctx);
}

void SitePin::tclGetRef(std::ostream& out, const Context *ctx) const {
    const std::string& site = get_site_name(ctx, this->bel).str(ctx);
    out << "get_site_pins -of_objects [get_sites " << extract_pure_site_name(site)
        << "] " << extract_pure_site_name(site) << "/"
        << get_bel_name(ctx, this->bel).c_str(ctx);
}

void DeclareCell::write(std::ostream& out, const Context *ctx) const {
    std::string pure_cell_name =
        extract_pure_cell_name(this->cell.cell->name.str(ctx));
    const std::string& type = this->cell.cell->type.str(ctx);

    std::string cmd;
    const char * direction = "NULL";

    switch (cell_kind(type)) {
        case CellKind::REGULAR:
            out << "create_cell -ref " << type << " " << pure_cell_name << "\n";
            return;
        case CellKind::IN_PAD:
            direction = "IN";
            break;
        case CellKind::OUT_PAD:
            direction = "OUT";
            break;
        case CellKind::IO_PAD:
            direction = "INOUT";
            break;
    }
    out << "create_port -direction " << direction << " " << pure_cell_name << "\n";
}

void PlaceCell::write(std::ostream& out, const Context *ctx) const {
    std::string pure_cell_name =
        extract_pure_cell_name(this->cell.cell->name.str(ctx));
    const std::string& type = this->cell.cell->type.str(ctx);

    IdString site = get_site_name(ctx, this->bel.bel);

    std::string cmd;
    CellKind kind = cell_kind(type);

    if (kind == CellKind::REGULAR) {
        std::string pure_site_name = extract_pure_site_name(site.str(ctx));
        out << "place_cell " << pure_cell_name << " " << pure_site_name << "/"
            << get_bel_name(ctx, this->bel.bel).str(ctx) << "\n";
        return;
    }

    if (this->cell.cell->bel != BelId()) {
        // XXX: We assume that the BEL is indeed bound correctly to the constrained port.
        auto iter = this->cell.cell->attrs.find(ctx->id("PACKAGE_PIN"));
        if (iter == this->cell.cell->attrs.end()) {
            iter = this->cell.cell->attrs.find(ctx->id("LOC"));
            if (iter == this->cell.cell->attrs.end()) {
                log_error("(NISP DUMP) Port '%s' is missing PACKAGE_PIN or LOC property\n",
                            this->cell.cell->name.c_str(ctx));
            }
        }
        out << "place_ports " << pure_cell_name << " " << iter->second.as_string() << "\n";
        return;
    }

    out <<  "# Port " << pure_cell_name << "is unconstrained. Placement is unclear.\n";
}

void CreateNet::write(std::ostream& out, const Context *ctx) const {
    std::string net_name = adjust_name_for_tcl(this->net.str(ctx));

    out << "create_net " << net_name << "\n";
}

void ConnectToNet::write(std::ostream& out, const Context *ctx) const {
    std::string net_name = adjust_name_for_tcl(this->net.str(ctx));

    std::string entries;

    for (size_t i = 0; i < this->obj_cnt; i++) {
        entries += cell_pin_name(this->objects[i], ctx);
        if (i != this->obj_cnt - 1) {
            entries += " ";
        }
    }

    out << "connect_net -net " << net_name << " -objects {" << entries << "}\n";
}

void vivado::Comment<std::string>::write(std::ostream& out, const Context *ctx) const {
    out << "# " << this->content << "\n";
}

void vivado::Comment<const char *>::write(std::ostream& out, const Context *ctx) const {
    out << "# " << this->content << "\n";
}

NEXTPNR_NAMESPACE_END
