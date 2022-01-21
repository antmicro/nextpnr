@0x97c69817483d9dea;

using Cxx = import "/capnp/c++.capnp";
$Cxx.namespace("lookahead_storage");

using DelayType = Int32;

struct CostMapEntry {
    key  @0 : Int32;
    data @1 : List(DelayType);
}

struct CostMap {
    costMap @0 : List(CostMapEntry);
}

struct Lookahead {
    costMap @0 : CostMap;
}

