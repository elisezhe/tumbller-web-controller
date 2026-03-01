/*
 * Wrapper around EnableInterrupt library for multi-file compilation.
 *
 * Include THIS header (not EnableInterrupt.h directly) from all .cpp files.
 * The actual implementation is compiled once in EnableInterruptImpl.cpp.
 */
#ifndef ENABLE_INTERRUPT_WRAPPER_H
#define ENABLE_INTERRUPT_WRAPPER_H

#define LIBCALL_ENABLEINTERRUPT
#include <EnableInterrupt.h>

#endif
