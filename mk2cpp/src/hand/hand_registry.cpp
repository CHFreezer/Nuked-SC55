/*
 * hand registry: collects per-module fill functions from static initializers.
 * Storage is a function-local static, so initialization order between
 * translation units is never an issue.
 */
#include "hand_registry.h"

namespace mk2c {
namespace {

const int kMaxHandModules = 64;

struct ModuleList {
    hand_fill_fn fn[kMaxHandModules];
    int n;
};

ModuleList &modules(void)
{
    static ModuleList list = {};
    return list;
}

} /* anonymous namespace */

void hand_register_module(hand_fill_fn fn)
{
    ModuleList &list = modules();
    if (fn != nullptr && list.n < kMaxHandModules)
        list.fn[list.n++] = fn;
}

void hand_fill_modules(void)
{
    ModuleList &list = modules();
    for (int i = 0; i < list.n; i++)
        list.fn[i]();
}

} /* namespace mk2c */
