#ifndef VERSION_H
#define VERSION_H

#include <stdint.h>

/*
 * Firmware (product) semantic version.
 *
 * It also gates the flash parameter store (app/params): on boot the stored
 * version is compared against this one and, if MAJOR.MINOR differs, the stored
 * parameters are discarded and reset to defaults. PATCH is ignored.
 *
 * Bump policy (what to change when):
 *   PATCH  - bugfixes, logic changes, ADDING a new parameter, reordering
 *            params_t fields, tweaking a default -> stored params PRESERVED.
 *   MINOR  - change the TYPE/SIZE or the MEANING/UNITS of an EXISTING stored
 *            parameter (i.e. old bytes would be misread or be semantically
 *            wrong) -> stored params auto-reset to defaults on the next boot.
 *   MAJOR  - major release -> also resets params (the gate is MAJOR.MINOR).
 */
#define FW_VER_MAJOR   0
#define FW_VER_MINOR   2
#define FW_VER_PATCH   13

/* Packed as 0x00MMmmpp (major/minor/patch, each 0..255). */
#define FW_VER_U32     (((uint32_t)FW_VER_MAJOR << 16) | \
                        ((uint32_t)FW_VER_MINOR << 8)  | \
                        ((uint32_t)FW_VER_PATCH))

/* MAJOR.MINOR only — the parameter-compatibility gate (top 16 bits). */
#define FW_VER_COMPAT  (FW_VER_U32 >> 8)

/* "MAJOR.MINOR.PATCH" string, built from the numbers so it can't drift. */
#define FW_VER_STR2(x) #x
#define FW_VER_STR(x)  FW_VER_STR2(x)
#define FW_VER_STRING  FW_VER_STR(FW_VER_MAJOR) "." FW_VER_STR(FW_VER_MINOR) "." FW_VER_STR(FW_VER_PATCH)

#endif /* VERSION_H */
