#ifndef VIVADO_H
#define VIVADO_H

#include <ostream>
#include <fstream>
#include <functional>

#include "nextpnr_types.h"

NEXTPNR_NAMESPACE_BEGIN
namespace vivado {

class TclCommand;

class TclWriter {
  public:
    TclWriter(std::string output_file_path, const Context *ctx)
        : out(output_file_path, std::ios_base::trunc)
        , ctx(ctx)
    {}
    void flush() {
        this->out.flush();
    }
    void close() {
        this->out.close();
    }
  protected:
    std::ofstream out;
    const Context *ctx;

    friend class TclCommand;
    friend TclWriter& operator << (TclWriter& writer, const TclCommand& cmd);
};

class DesignResource {
  public:
    virtual void tclGetRef(std::ostream& out, const Context *ctx) const = 0;
};

class Net : public DesignResource {
  public:
    Net(IdString name_) : name(name_) {}
    void tclGetRef(std::ostream& out, const Context *ctx) const override;
    IdString name;
};

class Site : public DesignResource {
  public:
    Site(IdString name_) : name(name_) {}
    void tclGetRef(std::ostream& out, const Context *ctx) const override;
    IdString name;
};

class Cell : public DesignResource {
  public:
    Cell(const CellInfo *cell_) : cell(cell_) {}
    void tclGetRef(std::ostream& out, const Context *ctx) const override;
    const CellInfo *cell;
};

class Bel : public DesignResource {
  public:
    Bel(BelId bel_) : bel(bel_) {}
    void tclGetRef(std::ostream& out, const Context *ctx) const override;
    BelId bel;
};

class SitePin : public DesignResource {
  public:
    SitePin(BelId bel_) : bel(bel_) {}
    void tclGetRef(std::ostream& out, const Context *ctx) const override;
    BelId bel;
};

class TclCommand {
  public:
    friend TclWriter& operator << (TclWriter& writer, const TclCommand& cmd) {
        cmd.write(writer.out, writer.ctx);
        return writer;
    }
    virtual ~TclCommand() {}
    virtual void write(std::ostream& out, const Context *ctx) const = 0;
};

class DeclareCell : public TclCommand {
  public:
    DeclareCell(Cell cell_, bool comment_ = false)
        : cell(cell_)
        , comment(comment_)
    {}
    ~DeclareCell() override = default;
    virtual void write(std::ostream& out, const Context *ctx) const override;
  private:
    Cell cell;
    bool comment;
};

class PlaceCell : public TclCommand {
  public:
    PlaceCell(Cell cell_, Bel bel_)
        : cell(cell_)
        , bel(bel_)
    {}
    ~PlaceCell() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    Cell cell;
    Bel bel;
};

class CreateNet : public TclCommand {
  public:
    CreateNet(IdString net_)
        : net(net_)
    {}
    ~CreateNet() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    IdString net;
    const std::string* objects = nullptr;
    size_t obj_cnt = 0;
};

class ConnectToNet : public TclCommand {
  public:
    ConnectToNet(IdString net_, std::vector<PortRef>& objects_)
        : net(net_)
        , objects(&objects_[0])
        , obj_cnt(objects_.size())
    {}
    ConnectToNet(IdString net_, PortRef& object)
        : net(net_)
        , objects(&object)
        , obj_cnt(1)
    {}
    ~ConnectToNet() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    IdString net;
    PortRef* objects = nullptr;
    size_t obj_cnt = 0;
};

enum class HighlightKind {
    COLOR,
    MARK,
};

enum class Color : size_t {
    RED,
    GREEN,
    BLUE,
    YELLOW,
    MAGENTA,
    __COUNT__
};

extern const char *vivado_highlight_color_map[];

template <typename T>
class Highlight : public TclCommand {
  public:
    Highlight(T resource_, Color color_, HighlightKind kind_ = HighlightKind::COLOR)
        : resource(resource_)
        , color(color_)
        , kind(kind_)
    {}
    ~Highlight() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    T resource;
    Color color;
    HighlightKind kind;
};

template <typename T>
Highlight<T> highlight(T resource, Color color,
                       HighlightKind kind = HighlightKind::COLOR)
{
    return Highlight<T>(resource, color, kind);
}

template <typename T>
class Comment : public TclCommand {
  public:
    Comment(T&& content_) : content(content_) {}
    ~Comment() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    T content;
};

template <size_t N>
class Comment<const char (&)[N]> : public TclCommand {
  public:
    Comment(const char (&content_)[N]) : content(content_) {}
    ~Comment() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    const char (&content)[N];
};

template <>
class Comment<std::string> : public TclCommand {
  public:
    Comment(std::string&& content_) : content(content_) {}
    ~Comment() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    std::string content;
};

template <>
class Comment<const char *> : public TclCommand {
  public:
    Comment(const char *content_) : content(content_) {}
    ~Comment() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    const char *content;
};

template <typename T>
Comment<T> comment(T&& content) {
    return Comment<T>(std::move(content));
}

template <typename C>
class AllowError : public TclCommand {
  public:
    AllowError(C&& command_) : command(command_) {}
    ~AllowError() override = default;
    void write(std::ostream& out, const Context *ctx) const override;
  private:
    C command;
};

template <typename C>
AllowError<C> allow_error(C&& command) {
    return AllowError<C>(std::move(command));
}

}
NEXTPNR_NAMESPACE_END

#endif /* VIVADO_H */
