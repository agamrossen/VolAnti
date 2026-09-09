/*
 * Debug.h - the vendored driver logs through this macro.
 *
 * It is a NO-OP here, on purpose. The trace stream is a binary record format
 * sharing one link with the log, and app_main silences ESP logging precisely
 * so a stray printf can never land inside a record. The e-paper module reports
 * through STAT records instead.
 */
#pragma once

#define Debug(...) do { } while (0)
