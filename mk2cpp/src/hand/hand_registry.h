#pragma once

namespace mk2c {

typedef void (*hand_fill_fn)(void);

/* Self-registration for hand modules (parallel safe): a module defines its fill
 * function and registers it from a file-static initializer, so adding a module
 * never edits a shared aggregator file. MK2CPP_HandFillTables calls
 * hand_fill_modules() after the built-in modules. */
void hand_register_module(hand_fill_fn fn);
void hand_fill_modules(void);

} /* namespace mk2c */
