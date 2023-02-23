#include "placer_repl.h"

#include "context.h"
#include "log.h"
#include "nextpnr.h"
#include "placer_heap.h"
#include "placer1.h"

#include <iostream>

// Include tcl.h late because it messed with #define's and lets them leave the
// scope of the header.
#include <tcl.h>

NEXTPNR_NAMESPACE_BEGIN


template <typename T>
struct RefObj;
/* This is what lack of partial function template specialization does to a programer */
template <typename T, bool Destructible>
struct __RefDropper {
    void drop(RefObj<T> *r);
};

template <typename T>
struct RefObj {
    union {
        T obj;
        ssize_t next_idx; /* For pool allocations */
    };
    size_t count;

    void take() {
        NPNR_ASSERT(this->count > 0);
        this->count++;
    }

    friend struct __RefDropper<T, std::is_destructible<T>::value>;

    void drop() {
        __RefDropper<T, std::is_destructible<T>::value>().drop(this);
    }

    T& operator*() {
        return this->obj;
    }

    RefObj() : next_idx(-1), count(0) {}
    RefObj(T&& o) : obj(o), count(1) {}
    ~RefObj() {
        if (this->count != 0) {
            log_warning("Tcl: destroyed object with non-sero reference count\n");
            this->~obj();
        }
    }
};

template <class T>
struct __RefDropper<T, true> {
    void drop(RefObj<T> *r) {
        NPNR_ASSERT(r->count > 0);
        r->count--;
        if (r->count == 0)
            r->obj.~T();
    }
};

template <class T>
struct __RefDropper<T, false> {
    void drop(RefObj<T> *r) {
        NPNR_ASSERT(r->count > 0);
        r->count--;
    }
};



// template <typename T>
// struct RefPool {
//     std::vector<RefObj<T>> refs;
//     ssize_t free = -1;
//     Context *ctx;
//
//     RefPool() {
//         this->refs.resize(4);
//
//
//
//         this->free = 0
//     }
//
//     RefObj<T>& asRef(T && obj) {
//         this->refs.push_back(RefObj {obj, 1});
//         this->
//     }
// };

template <typename T>
void ref_obj_free(Tcl_Obj *objPtr) {
    auto *obj = static_cast<RefObj<T> *>(objPtr->internalRep.twoPtrValue.ptr2);
    obj->drop();
    if (obj->count == 0) {
        obj->obj.~T();
        Tcl_Free((char *)obj);
    }
}

template <typename T>
void ref_obj_dup(Tcl_Obj *srcPtr, Tcl_Obj *dupPtr) {
    auto *obj = static_cast<RefObj<T> *>(srcPtr->internalRep.twoPtrValue.ptr2);
    obj->take();

    dupPtr->internalRep.twoPtrValue = srcPtr->internalRep.twoPtrValue;
}

// ---------------- Cell definition ----------------

static void obj_free(Tcl_Obj *objPtr);
static void twoptr_value_dup(Tcl_Obj *srcPtr, Tcl_Obj *dupPtr);
static void cell_update_string(Tcl_Obj *objPtr);
static int cell_set_from_any(Tcl_Interp *interp, Tcl_Obj *objPtr);

static Tcl_ObjType cell_object = {
    "cell", obj_free, twoptr_value_dup, cell_update_string, cell_set_from_any,
};

static void Tcl_SetStringResult(Tcl_Interp *interp, const std::string &s) {
    char *copy = Tcl_Alloc(s.size() + 1);
    std::copy(s.begin(), s.end(), copy);
    copy[s.size()] = '\0';
    Tcl_SetResult(interp, copy, TCL_DYNAMIC);
}

static int obj_set_from_any(Tcl_Interp *interp, Tcl_Obj *objPtr) { return TCL_ERROR; }

static void set_tcl_obj_string(Tcl_Obj *objPtr, const std::string &s) {
    NPNR_ASSERT(objPtr->bytes == nullptr);
    // Need to have space for the end null byte.
    objPtr->bytes = Tcl_Alloc(s.size() + 1);

    // Length is length of string, not including the end null byte.
    objPtr->length = s.size();

    std::copy(s.begin(), s.end(), objPtr->bytes);
    objPtr->bytes[objPtr->length] = '\0';
}

static void port_update_string(Tcl_Obj *objPtr) {
    const Context *ctx = static_cast<const Context *>(objPtr->internalRep.twoPtrValue.ptr1);
    PortInfo *port_info = static_cast<PortInfo *>(objPtr->internalRep.twoPtrValue.ptr2);

    std::string port_name = port_info->name.str(ctx);
    set_tcl_obj_string(objPtr, port_name);
}

static void cell_update_string(Tcl_Obj *objPtr) {
    const Context *ctx = static_cast<const Context *>(objPtr->internalRep.twoPtrValue.ptr1);
    CellInfo *cell_info = static_cast<CellInfo *>(objPtr->internalRep.twoPtrValue.ptr2);

    std::string cell_name = cell_info->name.str(ctx);
    set_tcl_obj_string(objPtr, cell_name);
}

static int cell_set_from_any(Tcl_Interp *interp, Tcl_Obj *objPtr) {
    if (objPtr->typePtr == &cell_object)
        return TCL_OK;

    const char* str = Tcl_GetString(objPtr);
    if (str == nullptr) {
        Tcl_SetStringResult(interp, "Can't convert object to string");
        return TCL_ERROR;
    }
    auto *ctx = static_cast<Context *>(Tcl_GetAssocData(interp, "ctx", nullptr));
    NPNR_ASSERT(ctx != nullptr);


    IdString obj_str = ctx->id(str);
    auto it = ctx->cells.find(obj_str);
    if (it == ctx->cells.end()) {
        Tcl_SetStringResult(interp, std::string("Can't find cell ") + str);
        return TCL_ERROR;
    }

    if (objPtr->typePtr != nullptr)
        objPtr->typePtr->freeIntRepProc(objPtr);
    objPtr->typePtr = &cell_object;
    objPtr->internalRep.twoPtrValue.ptr1 = (void *)ctx;
    objPtr->internalRep.twoPtrValue.ptr2 = (void *)&*it->second;

    return TCL_OK;
}

// ---------------- BEL definition ----------------

static void belid_refval_update_string(Tcl_Obj *objPtr);
static int belid_set_from_any(Tcl_Interp *interp, Tcl_Obj *objPtr);

static Tcl_ObjType bel_object = {
    "bel", ref_obj_free<BelId>, ref_obj_dup<BelId>, belid_refval_update_string, belid_set_from_any,
};

static void belid_refval_update_string(Tcl_Obj *objPtr) {
    const Context *ctx = static_cast<const Context *>(objPtr->internalRep.twoPtrValue.ptr1);

    auto& bel = *static_cast<RefObj<BelId> *>(objPtr->internalRep.twoPtrValue.ptr2);

    const BelInfoPOD &bel_pod = bel_info(ctx->chip_info, *bel);
    set_tcl_obj_string(objPtr, IdString(bel_pod.name).c_str(ctx));
}

static int belid_set_from_any(Tcl_Interp *interp, Tcl_Obj *objPtr) {
    if (objPtr->typePtr == &bel_object)
        return TCL_OK;

    const char* str = Tcl_GetString(objPtr);
    if (str == nullptr) {
        Tcl_SetStringResult(interp, "Can't convert object to string");
        return TCL_ERROR;
    }

    auto *ctx = static_cast<Context *>(Tcl_GetAssocData(interp, "ctx", nullptr));
    NPNR_ASSERT(ctx != nullptr);

    //IdString obj_str = ctx->id(str);
    // XXX: Nextpnr has a bit weird naming scheme for bels, so this needs to be taken
    // into account.
    BelId bel;
    try {
        bel = ctx->getBelByNameStr(str);
    } catch (std::out_of_range& e) {
        bel = BelId();
    }
    if (bel == BelId()) {
        Tcl_SetStringResult(interp, std::string("Bel ") + str + " not found");
        return TCL_ERROR;
    }

    auto *r_obj = Tcl_Alloc(sizeof(RefObj<BelId>));
    new(r_obj) RefObj<BelId>(std::move(bel));

    if (objPtr->typePtr != nullptr)
        objPtr->typePtr->freeIntRepProc(objPtr);
    objPtr->typePtr = &bel_object;
    objPtr->internalRep.twoPtrValue.ptr1 = (void *)ctx;
    objPtr->internalRep.twoPtrValue.ptr2 = (void *)r_obj;

    return TCL_OK;
}

static void twoptr_value_dup(Tcl_Obj *srcPtr, Tcl_Obj *dupPtr) {
    dupPtr->internalRep.twoPtrValue = srcPtr->internalRep.twoPtrValue;
}

static void obj_free(Tcl_Obj *objPtr) {}

static Tcl_ObjType port_object = {
    "port", obj_free, twoptr_value_dup, port_update_string, obj_set_from_any,
};


// Tcl REPL could work somewhat like that:
/*

> # successful bind
> bind $CELL $SLICE/BEL
true

> # unsuccessful bind
> bind $CELL $SLICE/BEL
false

> # unbind by cell
> unbind $CELL

> # unbind by bel
> unbind $BEL

> # get validity status for given cell (routable)
> status $CELL -routability
routable

> # get validity status for given bel (unroutable)
> status $BEL -routability
unroutable

> # get validity status for given bel (unbound)
> status $BEL -routability
unbound

> # get binding info
> get_bound_bel LUT4_30
SLICE_X63Y200/D5LUT
> get_bound_cell SLICE_X63Y200/D5LUT
LUT4_30

> # status information could be extended to various parameters, if needed eg.
> status $LUT5/O -routability
routable
> set conn_list [status LUT5_120/O -connections]
> lindex $conn_list 0
FF_abc/D
> set conn_dict [status LUT6_7q -connections]
> [dict get $conn_dict $CELL/PORT]
FF_aaa/D FF_1du/D

*/

static int tcl_get_cell_name(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 2) {
        Tcl_SetStringResult(interp, "Expected one argument. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    Tcl_Obj *obj = objv[1];
    int error;
    if ((error = Tcl_ConvertToType(interp, obj, &cell_object)) != TCL_OK)
        return error;
    const Context *ctx = static_cast<const Context *>(obj->internalRep.twoPtrValue.ptr1);
    auto *cell = static_cast<CellInfo *>(obj->internalRep.twoPtrValue.ptr2);

    Tcl_SetStringResult(interp, cell->name.str(ctx));
    return TCL_OK;
}

static int tcl_get_cells(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[])
{
    Context *ctx = static_cast<Context *>(data);

    auto create_cell_obj = [&](CellInfo *cell) -> Tcl_Obj * {
        Tcl_Obj *obj = Tcl_NewObj();

        obj->typePtr = &cell_object;
        obj->internalRep.twoPtrValue.ptr1 = (void *)ctx;
        obj->internalRep.twoPtrValue.ptr2 = (void *)cell;
        obj->bytes = nullptr;

        //cell_update_string(obj);

        return obj;
    };

    if (objc == 1) {
        Tcl_Obj *cells_obj = Tcl_NewListObj(ctx->cells.size(), nullptr);
        for (auto& cell_kv : ctx->cells) {
            CellInfo *cell = cell_kv.second.get();
            Tcl_ListObjAppendElement(interp, cells_obj, create_cell_obj(cell));
        }

        Tcl_SetObjResult(interp, cells_obj);
        return TCL_OK;
    } else if (objc == 2) {
        const char *arg0 = Tcl_GetString(objv[1]);
        IdString cell_name = ctx->id(arg0);

        auto iter = ctx->cells.find(cell_name);
        if (iter == ctx->cells.end()) {
            Tcl_SetStringResult(interp, "Could not find cell " + cell_name.str(ctx));
            return TCL_ERROR;
        }

        Tcl_Obj *result = create_cell_obj(iter->second.get());
        Tcl_SetObjResult(interp, result);
        return TCL_OK;
    } else if (objc > 2) {
        log_warning("get_cells options not implemented!\n");
        return TCL_OK;
    } else {
        return TCL_ERROR;
    }
}

static int tcl_get_bels(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *CONST objv[]) {
    if (objc != 1) {
        Tcl_SetStringResult(interp, "Expected zero arguments. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    const Context *ctx = static_cast<const Context *>(data);

    Tcl_Obj *bels_obj = Tcl_NewListObj(2048, nullptr);
    for (auto bel : ctx->getBels()) {
        auto *r_obj = Tcl_Alloc(sizeof(RefObj<BelId>));
        new(r_obj) RefObj<BelId>(std::move(bel));

        Tcl_Obj *obj = Tcl_NewObj();

        obj->typePtr = &bel_object;
        obj->internalRep.twoPtrValue.ptr1 = (void *)ctx;
        obj->internalRep.twoPtrValue.ptr2 = (void *)r_obj;
        obj->bytes = nullptr;

        Tcl_ListObjAppendElement(interp, bels_obj, obj);
    }

    Tcl_SetObjResult(interp, bels_obj);

    return TCL_OK;
}

static int tcl_get_bel_name(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 2) {
        Tcl_SetStringResult(interp, "Expected one argument. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    Tcl_Obj *obj = objv[1];
    int error;
    if ((error = Tcl_ConvertToType(interp, obj, &bel_object)) != TCL_OK)
        return error;
    const Context *ctx = static_cast<const Context *>(obj->internalRep.twoPtrValue.ptr1);
    auto& bel = *static_cast<RefObj<BelId> *>(obj->internalRep.twoPtrValue.ptr2);

    Tcl_SetStringResult(interp, ctx->getBelName(*bel).str(ctx));
    return TCL_OK;
}

static int tcl_bind(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    int error;

    if (objc != 3) {
        Tcl_SetStringResult(interp, "Expected two arguments (<cell> <bel>). Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    Context *ctx = static_cast<Context *>(data);
    Tcl_Obj *cell_obj = objv[1];
    Tcl_Obj *bel_obj = objv[2];

    if ((error = Tcl_ConvertToType(interp, cell_obj, &cell_object)) != TCL_OK)
        return error;
    if ((error = Tcl_ConvertToType(interp, bel_obj, &bel_object)) != TCL_OK)
        return error;

    auto *cell = static_cast<CellInfo *>(cell_obj->internalRep.twoPtrValue.ptr2);
    auto& bel = *static_cast<RefObj<BelId> *>(bel_obj->internalRep.twoPtrValue.ptr2);

    NPNR_ASSERT(*bel != BelId());

    ctx->bindBel(*bel, cell, PlaceStrength::STRENGTH_USER);

#ifdef ENABLE_NISP
    const auto& bind_info = ctx->nisp_handler.getCellBindInfo(cell);
    Tcl_SetStringResult(interp,
                        bind_info.success ? "NISP: routable" : "NISP: unroutable");
#endif

    return TCL_OK;
}

static int tcl_unbind(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    int error;

    if (objc != 2) {
        Tcl_SetStringResult(interp, "Expected one argument (<bel>). Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    Context *ctx = static_cast<Context *>(data);
    Tcl_Obj *bel_obj = objv[1];
    if ((error = Tcl_ConvertToType(interp, bel_obj, &bel_object)) != TCL_OK)
        return error;
    auto& bel = *static_cast<RefObj<BelId> *>(bel_obj->internalRep.twoPtrValue.ptr2);

    ctx->unbindBel(*bel);

    return TCL_OK;
}

static int tcl_dump_design(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if ((objc < 2) || (objc > 3)) {
        Tcl_SetStringResult(interp, "one argument (<path>), found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    std::string path = Tcl_GetString(objv[1]);

    // FIXME: Linker gives up when trying to call this method directly.
    Context *ctx = static_cast<Context *>(data);
    Arch* arch_ptr = static_cast<Arch*>(ctx);
    arch_ptr->dumpDesignStateToTcl(path);

    return TCL_OK;
}

static int tcl_place_constraints(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 1) {
        Tcl_SetStringResult(interp, "Expected zero arguments. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    static_cast<Context *>(data)->place_constraints();

    return TCL_OK;
}

static int tcl_place_globals(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 1) {
        Tcl_SetStringResult(interp, "Expected zero arguments. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    static_cast<Context *>(data)->place_globals();

    return TCL_OK;
}

// TODO: Allow placer configuration through command arguments
static int tcl_place_heap(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 1) {
        Tcl_SetStringResult(interp, "Expected zero arguments. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    auto *ctx = static_cast<Context *>(data);

    PlacerHeapCfg cfg(ctx);
    cfg.criticalityExponent = 7;
    cfg.alpha = 0.08;
    cfg.beta = 0.4;
    cfg.placeAllAtOnce = true;
    cfg.hpwl_scale_x = 1;
    cfg.hpwl_scale_y = 2;
    cfg.spread_scale_x = 2;
    cfg.spread_scale_y = 1;
    cfg.solverTolerance = 0.6e-6;

    if (!placer_heap(ctx, cfg)) {
        Tcl_SetStringResult(interp, "HeAP: FAILURE");
        return TCL_ERROR;
    }

    Tcl_SetStringResult(interp, "HeAP: SUCCESS");
    return TCL_OK;
}

static int tcl_place_sa(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 1) {
        Tcl_SetStringResult(interp, "Expected zero arguments. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    auto *ctx = static_cast<Context *>(data);

    if (!placer1(ctx, Placer1Cfg(ctx))) {
        Tcl_SetStringResult(interp, "SA: FAILURE");
        return TCL_ERROR;
    }

    Tcl_SetStringResult(interp, "SA: SUCCESS");
    return TCL_OK;

    static_cast<Context *>(data)->place_globals();

    return TCL_OK;
}

static int tcl_place_check(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    if (objc != 1) {
        Tcl_SetStringResult(interp, "Expected zero arguments. Found " + std::to_string(objc - 1));
        return TCL_ERROR;
    }

    auto *ctx = static_cast<Context *>(data);

    // TODO: This will exit nextpnr on failure. Context::check should be modified.
    ctx->check();

    Tcl_SetStringResult(interp, "true");
    return TCL_OK;
}

static int tcl_done(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    bool* do_exit = (bool *)data;
    *do_exit = true;

    return TCL_OK;
}

static int tcl_exit(ClientData data, Tcl_Interp *interp, int objc, Tcl_Obj *const objv[]) {
    Tcl_SetStringResult(interp, "`exit` command is not available in this context. "
                                "Use `done` to end manual placement.");
    return TCL_ERROR;
}

#ifdef ENABLE_NISP
static int tcl_nisp_stats(ClientData data, Tcl_Interp *interp, int objc,
                          Tcl_Obj *const objv[]) {
    const auto *ctx = static_cast<const Context *>(data);

    Tcl_Obj *d = Tcl_NewDictObj();

    static const char *keys[] = {
        "totalChecks",
        "misses",
        "falsePositives",
        "falseNegatives"
    };

    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("totalChecks", -1),
                   Tcl_NewIntObj((int)ctx->nisp_handler.totalChecks));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("misses", -1),
                   Tcl_NewIntObj((int)ctx->nisp_handler.misses));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("falsePositives", -1),
                   Tcl_NewIntObj((int)ctx->nisp_handler.falsePositives));
    Tcl_DictObjPut(interp, d, Tcl_NewStringObj("falseNegatives", -1),
                   Tcl_NewIntObj((int)ctx->nisp_handler.falseNegatives));

    Tcl_SetObjResult(interp, d);

    return TCL_OK;
}
#else /* ENABLE_NISP */
static int tcl_nisp_stats(ClientData data, Tcl_Interp *interp, int objc,
                          Tcl_Obj *const objv[]) {
    Tcl_SetStringResult(interp, "nextpnr has been compiled without NISP support");
    return TCL_ERROR;
}
#endif /* ENABLE_NISP */

bool placer_repl(Context *ctx) {
    auto *interp = Tcl_CreateInterp();
    auto r = Tcl_Init(interp);
    NPNR_ASSERT(r == TCL_OK);

    bool do_exit = false;

    Tcl_SetAssocData(interp, "ctx", nullptr, ctx);
    Tcl_InterpDeleteProc p;

    Tcl_RegisterObjType(&port_object);
    Tcl_RegisterObjType(&cell_object);
    Tcl_RegisterObjType(&bel_object);

    Tcl_CreateObjCommand(interp, "get_cells", tcl_get_cells, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "get_bels", tcl_get_bels, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "get_cell_name", tcl_get_cell_name, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "get_bel_name", tcl_get_bel_name, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "bind", tcl_bind, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "unbind", tcl_unbind, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "dump_design", tcl_dump_design, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "place_constraints", tcl_place_constraints, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "place_globals", tcl_place_globals, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "place_heap", tcl_place_heap, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "place_sa", tcl_place_sa, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "place_check", tcl_place_check, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "nisp_stats", tcl_nisp_stats, ctx, nullptr);
    Tcl_CreateObjCommand(interp, "done", tcl_done, &do_exit, nullptr);
    Tcl_CreateObjCommand(interp, "exit", tcl_exit, &do_exit, nullptr);

    const char init_script[] =
        "proc place_initial {} {\n"
        "    place_constraints\n"
        "    place_globals\n"
        "}\n";

    Tcl_EvalEx(interp, init_script, -1, TCL_EVAL_DIRECT);

    std::string line, cmd;

    std::cout << "------ ACTIVATED REPL PLACEMENT MODE ------" << std::endl;

    do {
        std::cout << "[place@initial]# " << std::flush;

        cmd.clear();

        while (true) {
            if (std::getline(std::cin, line).eof()) {
                log_warning("ERROR READING LINE. EXITING REPL MODE\n");
                do_exit = true;
                break;
            }

            cmd += line;

            if (Tcl_CommandComplete(cmd.c_str()))
                break;

            std::cout << "            >>>  " << std::flush;
        }

        // TODO: Input should be UTF-8, but this is not guranteed here.
        if (Tcl_EvalEx(interp, cmd.c_str(), -1, TCL_EVAL_DIRECT) != TCL_OK) {
            const char* err_str = Tcl_GetStringResult(interp);
            if (err_str != nullptr)
                std::cout << "[!] ERROR: " <<  err_str << std::endl;
            else
                std::cout << "[!] ERROR: <unkown error>" << std::endl;
        } else {
            Tcl_Obj *ret_obj = Tcl_GetObjResult(interp);
            if (ret_obj != nullptr) {
                const char* objstr = Tcl_GetString(ret_obj);
                if (strlen(objstr) != 0)
                    std::cout << objstr << std::endl;
            }
        }
    } while (!do_exit);

    std::cout << "--------- FINISHED REPL PLACEMENT ---------" << std::endl;

    Tcl_DeleteInterp(interp);

    return true;
}

NEXTPNR_NAMESPACE_END
