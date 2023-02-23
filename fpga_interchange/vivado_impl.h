#ifndef VIVADO_IMPL_H
#define VIVADO_IMPL_H

#include "vivado.h"


NEXTPNR_NAMESPACE_BEGIN

template <typename T>
void vivado::Highlight<T>::write(std::ostream& out, const Context *ctx) const {
        NPNR_ASSERT(this->color < Color::__COUNT__);
        if (kind == HighlightKind::MARK)
            out << "mark_objects [";
        else
            out << "highlight_objects [";
        this->resource.tclGetRef(out, ctx);
        out <<  "] -color " << vivado_highlight_color_map[(size_t)this->color] << "\n";
    }

template <size_t N>
void vivado::Comment<const char (&)[N]>::write(std::ostream& out, const Context *ctx) const {
    out << "# " << this->content << "\n";
}

template <typename T>
void vivado::Comment<T>::write(std::ostream& out, const Context *ctx) const {
    out << "# ";
    this->content.write(out, ctx);
}

template <typename C>
void vivado::AllowError<C>::write(std::ostream& out, const Context *ctx) const {
    out << "if {[catch {";
    this->command.write(out, ctx);
    out << "} errorstring]} { puts \"ERROR: $errorstring\" }\n";
}


NEXTPNR_NAMESPACE_END

#endif
